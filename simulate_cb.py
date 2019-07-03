import serial
import progressbar
import argparse
import sys
import binascii
import re
from serial import Serial
from time import sleep
from time import time


CB_CONNECTED = 1
CB_DISCONNECT = 2

TIMEOUT = 120
ISCA_LOST = -5
ISCA_RECOVERED = -6
ATTEMPTS_FAILED = -7

SERVICE_ID = '00000001100020003000111122223333'

addr = 0

BATTERY_ADDR = '0x0023'
TID_ADDR = '0x0026'
OP_MODE_CTRL_ADDR = '0x0029'
OP_MODE_VAL_ADDR = '0x002C'
STAT_SLEEP_TIME_ADDR = '0x0030'
STAT_WAIT_TIME_ADDR = '0x0033'
DYN_SLEEP_TIME_ADDR = '0x0036'
DYN_WAIT_TIME_ADDR = '0x0039'

class UnexpectedDisconnect(Exception) :
    pass

class UartDriver :
    def __init__(self, baud=115200, port="/dev/ttyUSB0", timeout=0.5, verbose=0) :
        self.baud = baud
        self.port = port
        self.verbose = int(verbose)
        self.serial = None
        self.full_response = ""
        self.bind()

    def write(self, data) :
        if self.verbose: 
            print("Enviando \"%s\"" % data, end='')
        self.serial.write(data.encode('ascii'))
        response = self.serial.read(300)
        if self.verbose: 
            print("Resposta do comando: \"%s\"" % response.decode('ascii'))
            print("++++++")
        return response

    def read(self) :
        msg = self.mount_msg('r', '')
        return self.write(msg)

    def read_until_x(self, timeout=1, pattern=None) :
        now = int(time())
        last = now
        self.full_response = ""

        if pattern == None :
            while now - last <= timeout :
                response = self.read()
                self.full_response += response.decode('ascii')
                self.clear_buffer()
                if response == b'\r\nbro "" FFFF\r\n' :
                    return 0
                now = int(time())
        else :
            while now - last <= timeout :
                response = self.read()
                self.clear_buffer()
                response = response.decode('ascii')
                self.full_response += response
                matches = re.findall(pattern, response)
                if len(matches) :
                    return 0
                elif pattern != "CONNECT" and pattern != "NO CARRIER" and pattern != "NO CARRIER|OK" and len(re.findall("NO CARRIER", response)) :
                    raise UnexpectedDisconnect("Unexpected disconnect!")
                elif pattern == "CONNECT" and len(re.findall("NO CARRIER", response)) :
                   return ISCA_LOST 
                elif pattern == "NO CARRIER" and len(re.findall("OK", response)) :
                    return ISCA_RECOVERED
                elif len(re.findall("SSPPIN \w*,t[1-3] \?", response)):
                    msg = "AT+BSSPPIN " + re.findall("\w*,t[1-3]", response)[0] + ",123456"
                    self.transport_msg(attempts=2, timeout=10, data=msg, expected="OK")
                    print("Aguardando a isca processar o bound...")
                    sleep(30)
                now = int(time())

        return 1

    def clear_buffer(self) :
        msg = self.mount_msg('a', '')
        self.write(msg)

    def mount_msg(self, cmd, suffix) :
        if cmd == 'a' or cmd == 'r':
            data = "b "
            data += cmd
            data += "\r\n"
        elif cmd == 'w':
            data = "b "
            data += cmd + " \""
            data += suffix
            data += "\" "
            crc = binascii.crc_hqx(suffix.encode('ascii'), 0xFFFF)
            data += '{:04x}'.format(crc)
            data += "\r\n"

        return data


    def bind(self) :
        print("[UART_BIND][ ]: The uart driver will be bind to port %s with baudrate %d" % (self.port, self.baud))
        try :
            self.serial = Serial(port=self.port, baudrate=self.baud, bytesize=8, timeout=2, stopbits=serial.STOPBITS_ONE)
        except :
            print("Error bind uart to port %s with baudrate %d" % (self.port, self.baud))
        print("[UART_BIND][X]")

    def transport_msg(self, attempts, timeout, data, expected="OK") :
        err = 0
        data = self.mount_msg('w', data)
        while attempts > 0 :
            self.write(data)
            err = self.read_until_x(timeout, expected)
            if not err :
                return 0
            elif err == ISCA_LOST :
                return ISCA_LOST
            elif err == ISCA_RECOVERED :
                return ISCA_RECOVERED
            attempts = attempts - 1

        return ATTEMPTS_FAILED


class Simulation :
    def __init__(self, uart, mode, setup) :
        if not uart or not mode :
            raise Exception("Uart device or mode was not specified!")
        self.setup = setup
        self.uart = uart
        self.scenario = {'static': self.super_static,
                         'recover_dyn': self.recover_dyn,
                         'bait_lost': self.bait_lost,
                         'field' : self.bait_in_field}
        self.mode = mode
        self.now_tid = int(time())
        self.setup_arr = ['AT+LETIO=4', 'AT+BIOCAP=2',
                         'AT+BMITM=1', 'AT+LECONINTMIN=400',
                          'AT+LECONINTMAX=400', 'AT+LEROLE=1',
                          'AT&W', 'AT+RESET']

    def cb_setup(self) :
        print("[CB_SETUP][ ] : Setando as configurações necessárias para o extensor atuar como CB")

        for i in progressbar.progressbar(range(len(self.setup_arr)), redirect_stdout=True) :
            print("Comando %s enviado\n" % self.setup_arr[i])
            self.uart.transport_msg(attempts=1, timeout=10, data=self.setup_arr[i], expected="OK")
            if self.setup_arr[i] == 'AT+RESET' :
                sleep(5)

        print("[CB_SETUP][X]")


    def connect(self, addr) :
        data = 'ATD'
        data += addr
        data += ',GATT'
        return self.uart.transport_msg(attempts=1, timeout=10, data=data, expected="CONNECT")

    def read_service(self, channel) :
        msg = 'AT+LEREAD=0x10,' + channel
        expected = 'LEREAD:0x10,' + channel
        err = self.uart.transport_msg(attempts=2, timeout=10, data=msg, expected=expected)

    def set_times(self, static_sleep_time, static_wait_time, dyn_sleep_time, dyn_wait_time) :
        time_msgs = ['AT+LEWRITE=0x10,' + STAT_SLEEP_TIME_ADDR + (",%04x" % static_sleep_time),
                     'AT+LEWRITE=0x10,' + STAT_WAIT_TIME_ADDR + (",%04x" % static_wait_time),
                     'AT+LEWRITE=0x10,' + DYN_SLEEP_TIME_ADDR + (",%04x" % dyn_sleep_time),
                     'AT+LEWRITE=0x10,' + DYN_WAIT_TIME_ADDR + (",%04x" % dyn_wait_time)
                     ]

        for i in progressbar.progressbar(range(len(time_msgs)), redirect_stdout=True) :
            print("Enviando comando %s\n" % time_msgs[i])
            err = self.uart.transport_msg(attempts=1, timeout=10, data=time_msgs[i], expected="OK")
            if err:
                return err

        return 0

    def show_baits(self) :
        content = self.uart.full_response
        content = content.replace('bro', '')
        content = re.sub('\" [A-Z0-9]{4}', '', content)
        content = content.replace('\"', '\b')
        temp = content.split('\n')
        content = ""
        for x in temp :
            content += x
        content = content.split()

        self.indexs = []
        self.names = []
        self.addrs = []

        for i in range(len(content)) :
            if len(re.findall("ISCA", content[i])) :
                self.indexs.append(i)
                self.names.append(content[i] + " " + content[i+1] + " " + content[i+2])

        for x in self.indexs :
            found = False
            while not found and x > 0:
                x -= 1
                if len(re.findall("[A-Z0-9]{12},t2", content[x])) :
                    self.addrs.append(content[x])
                    found = True
            if not x :
                print("erro legal")

        if len(self.indexs) :
            for i in range(len(self.indexs)) :
                print("%d: %s [%s]" % (i+1, self.addrs[i], self.names[i]))
        else :
            print("Nenhuma isca encontrada!")

    def search_bait(self) :
        print("[PESQUISANDO][ ]: Por favor, escolha o endereço da isca ao fim da pesquisa")
        addr = ""
        end = False
        while(not end) :
            self.uart.transport_msg(attempts=1, timeout=20, data="AT+LESCAN=GATT")
            self.show_baits()
            print("Digite o índice da isca a ser escolhida ou -1 para continuar pesquisando")
            idx = int(input("ÍNDICE: "))
            if (idx != -1) :
                print("%s escolhida!" % self.names[idx-1])
                addr = self.addrs[idx-1]
                end = True
        print("[PESQUISANDO][X]")

        return addr

    def send_tid(self, expected=None) :
        tid_msg = 'AT+LEWRITE=0x10,' + TID_ADDR + (",000F")
        self.actual_state = CB_DISCONNECT
        self.now_tid = int(time())
        if expected == None :
            return self.uart.transport_msg(attempts=1, timeout=10, data=tid_msg, expected="NO CARRIER")
        else :
            return self.uart.transport_msg(attempts=1, timeout=10, data=tid_msg, expected=expected)

    def change_mode(self, mode) :
        opmodectrl_msg = 'AT+LEWRITE=0x10,' + OP_MODE_CTRL_ADDR + "," + mode
        return self.uart.transport_msg(attempts=2, timeout=10, data=opmodectrl_msg, expected="OK")

    def super_static(self) :
        sleep_time = int(input("Type the sleep static time that you want: "))
        wait = int(input("Type the wait static time that you want: "))
        it_to_finish = int(input("Type the number of iterations to trip finish: "))
        end = False

        while not end :
            try : 
                addr = self.search_bait()

                err = self.connect(addr)
                if (err) :
                    print("Error connecting to bait")
                    return err

                err = self.set_times(sleep_time=sleep_time, wait=wait)
                if (err) :
                    print("Error setting times")
                    return err

                err = self.change_mode("01")
                if (err) :
                    print("Error changing to supervised static mode")
                    return err

                while(it_to_finish > 0) :
                    self.send_tid()
                    now = int(time())
                    print("Waiting for wakeup bait...\nsleep_time = %d\nnow = %d\nnow_tid = %d\ndiff = %d" % (sleep_time, now, self.now_tid, now - self.now_tid))
                    waiting_bait = sleep_time - (now - self.now_tid) if sleep_time - (now - self.now_tid) > 0 else 0
                    sleep(waiting_bait)
                    self.connect(addr)
                    it_to_finish -= 1

                self.change_mode("00")
                end = True

                err = self.disconnect("0x10")
                if (err) :
                    print("Error disconnecting to bait")
                    return err
                print("Test performed successfully")

            except UnexpectedDisconnect as ud:
                print("Unexpected disconnect")
                print("Repeating routine...")
                

    def disconnect(self, channel) :
        msg = 'ATH=' + channel
        return self.uart.transport_msg(attempts=2, timeout=10, data=msg, expected="NO CARRIER|OK")
        
    def recover_dyn(self) :
        sleep_time = int(input("Type the last sleep static time: "))
        wait = int(input("Type the last wait static time: "))
        it_to_finish = int(input("Type the number of iterations to trip finish: "))
        end = False

        err = self.disconnect("0x10")
        if (err) :
            print("Error disconnecting to bait")
            return err

        while (not end) :
            try : 
                addr = self.search_bait()
                err = self.connect(addr)
                if (err) :
                    print("Error connecting to bait")
                    return err

                self.send_tid("OK")

                while (it_to_finish) :
                    self.send_tid()
                    print("Waiting for wakeup bait...")
                    now = int(time())
                    waiting_bait = sleep_time - (now - self.now_tid) if sleep_time - (now - self.now_tid) > 0 else 0
                    sleep(waiting_bait)
                    self.connect(addr)
                    it_to_finish -= 1

                self.change_mode("00")
                err = self.disconnect("0x10")
                if (err) :
                    print("Error disconnecting to bait")
                    return err
                end = True
                print("Test performed successfully")
            except UnexpectedDisconnect as ud :
                print("Unexpected disconnect")
                print("Repeating routine...")
                


    def bait_lost(self) :
        sleep_time = int(input("Type the sleep static time that you want: "))
        wait = int(input("Type the wait static time that you want: "))
        static_it = int(input("Max static iterations: "))
        lost_time = int(input("Type the lost time: "))
        it_to_finish = int(input("Type the number of iterations to trip finish: "))
        end = False

        while not end :
            try :
                err = self.disconnect("0x10")
                if (err) :
                    print("Error disconnecting to bait")
                    return err

                addr = self.search_bait()

                err = self.connect(addr)
                if (err) :
                    print("Error connecting to bait")
                    return err

                err = self.set_times(sleep_time=sleep_time, wait=wait)
                if (err) :
                    print("Error setting times")
                    return err

                err = self.change_mode("01")
                if (err) :
                    print("Error changing to supervised static mode")
                    return err

                while(static_it > 0) :
                    self.send_tid()
                    print("Waiting for wakeup bait...")
                    now = int(time())
                    waiting_bait = sleep_time - (now - self.now_tid) if sleep_time - (now - self.now_tid) > 0 else 0
                    sleep(waiting_bait)
                    self.connect(addr)
                    static_it -= 1


                err = self.disconnect("0x10")
                if (err) :
                    print("Error disconnecting to bait")
                    return err

                print("Bait lost...")
                sleep(lost_time)
                print("Let's save it")

                addr = self.search_bait()

                err = self.connect(addr)
                if (err) :
                    print("Error connecting to bait")
                    return err

                self.send_tid("OK")

                while (it_to_finish > 0) :
                    self.send_tid()
                    print("Waiting for wakeup bait...")
                    now = int(time())
                    waiting_bait = sleep_time - (now - self.now_tid) if sleep_time - (now - self.now_tid) > 0 else 0
                    sleep(waiting_bait)
                    self.connect(addr)
                    it_to_finish -= 1

                self.change_mode("00")

                err = self.disconnect("0x10")
                if (err) :
                    print("Error disconnecting to bait")
                    return err
                
                print("Test performed successfully")
                end = True
            except UnexpectedDisconnect as ud :
                print("Unexpected disconnect")
                print("Repeating routine...")

    def bounding(self) :
        addr = self.addr
        addr = addr.replace(",t2", "")
        print("Verificando se o processo de bound já foi realizado com a isca %s..." % addr)
        msg = "AT+BNDLIST"
        if self.uart.transport_msg(attempts=1, timeout=5, data=msg, expected=addr) :
            print("Iniciando processo de bound com a isca...")
            msg = "AT+LEREAD=0x10,0x0033"
            self.uart.transport_msg(attempts=2, timeout=50, data=msg, expected="OK")
            print("Bound realizado com sucesso!")
        else :
            print("Processo de bound já foi realizado!")

    def bait_in_field(self) :
        print("Limpando buffer...")
        while self.uart.read_until_x() :
            pass
        print("Buffer limpo!")

        print("Desconectando de possíveis antigas conexões...")
        self.disconnect("0x10")
        print("Extensor desconectado de todas as suas conexões!")

        self.addr = self.search_bait()
        static_sleep_time = int(input("Tempo de suspensão da isca no modo estático: "))
        static_wait_time = int(input("Tempo de espera da isca no modo estático: "))
        dyn_sleep_time = int(input("Tempo de suspensão da isca no modo dinâmico: "))
        dyn_wait_time = int(input("Tempo de espera da isca no modo dinâmico: "))
        nisca_lost = 0
        end = False
        setup = True
        self.actual_state = CB_DISCONNECT

        while not end :
            try :
                print("Tentando conectar à isca...")
                while(self.connect(self.addr)) :
                    print("Tentando conectar à isca...")

                self.actual_state = CB_CONNECTED
                print("CB conectado à Isca!")

                if setup :

                    self.bounding()

                    print("Configurando tempos da isca...")
                    err = self.set_times(static_sleep_time=static_sleep_time, static_wait_time=static_wait_time,
                                         dyn_sleep_time=dyn_sleep_time, dyn_wait_time=dyn_wait_time)
                    if (err) :
                        print("Error setting times")
                        return err
                    print("Tempos configurados com sucesso!")

                    err = self.change_mode("01")
                    if (err) :
                        print("Error changing to supervised static mode")
                        return err

                    print("Mudando para o modo estático!")

                    normal = False

                while(True) :
                    print("Tentando estabelecer comunicação via TID...")
                    while self.send_tid() :
                        pass
                    print("Comunicação estabelecida com sucesso!\nConexão perdida com a isca!")
                    now = int(time())
                    print("Isca entrou em modo low power, esperando ela acordar para tentar conexão...")
                    waiting_bait = static_sleep_time - (now - self.now_tid) if static_sleep_time - (now - self.now_tid) > 0 else 0
                    sleep(waiting_bait)
                    print("Tentando conectar com a isca...")
                    err = self.connect(self.addr)
                    while err :
                        if err == ISCA_LOST :
                            print("Isca perdida, tentando reconectar...")
                            nisca_lost = 1
                        err = self.connect(self.addr)
                    print("CB conectado à isca!")
                    self.actual_state = CB_CONNECTED
                            
            except UnexpectedDisconnect as ud :
                print("Isca perdida!!!")
                nisca_lost = 1
        

    def run(self) :
        if self.setup :
            self.cb_setup()

        # data = 'AT+LESRVD=0x10,u00000001100020003000111122223333'
        # data = self.uart.mount_msg('w', data)
        # self.uart.write(data)
        # sleep(5)
        # self.read_until_noth()

        # while True :
            # pass

        try : 
            self.scenario[self.mode]()
        except KeyboardInterrupt :
            if self.actual_state == CB_CONNECTED :
                print("Encerrando viagem...")
                self.change_mode("00")
                print("Viagem encerrada com sucesso!")
                self.disconnect("0x10")
                print("Desconectando da isca!")

            elif self.actual_state == CB_DISCONNECT :
                print("Isca desconectada, tentando conectar com a isca para finalizar viagem...")
                err = self.connect(self.addr)
                while err :
                    if err == ISCA_LOST :
                        print("Isca perdida, tentando reconectar...")
                        nisca_lost = 1
                    err = self.connect(self.addr)
                print("CB conectado à isca!")
                print("Encerrando viagem...")
                self.change_mode("00")
                print("Viagem encerrada com sucesso!")
                self.disconnect("0x10")
                print("Desconectando da isca!")

def main() :
    parser = argparse.ArgumentParser(description='Tracker working as CB')
    parser.add_argument('-b', '--baudrate', default=115200,
                        help='UART baudrate')
    parser.add_argument('-p', '--port', type=str,
                        help='Port of UART device', default="/dev/ttyUSB0")
    parser.add_argument('-m', '--mode', type=str,
                        help='Simulation mode', default="")
    parser.add_argument('-s', '--setup', default=0,
                        help='Enable the setup to extensor work as CB')
    parser.add_argument('-v', '--verbose', default=0,
                        help='Simulation verbose')
    args = parser.parse_args(sys.argv[1:])

    print("**************************************")
    print("*       Tracker as Control Board     *")
    print("**************************************")

    uart = UartDriver(baud=args.baudrate, port=args.port, verbose=args.verbose)
    s = Simulation(uart=uart, mode=args.mode, setup=int(args.setup))
    s.run()


if __name__ == "__main__":
    main()
