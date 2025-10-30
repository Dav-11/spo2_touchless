package main

import (
	"fmt"
	"go.bug.st/serial"
	"log"
)

const SerialPort = "/dev/tty.usbserial-110"

func main() {

	mode := &serial.Mode{
		BaudRate: 115200,          // your sketch's baud rate
		Parity:   serial.NoParity, // default config
		StopBits: serial.OneStopBit,
		DataBits: 8,
	}

	port, err := serial.Open(SerialPort, mode)
	if err != nil {
		log.Fatal(err)
	}
	defer port.Close()

	// DO NOT toggle DTR/RTS — just read
	port.SetDTR(false)
	port.SetRTS(false)

	fmt.Println("ESP32-CAM connected, waiting for data...")

	buf := make([]byte, 512)
	for {
		n, err := port.Read(buf)
		if err != nil {
			log.Fatal(err)
		}
		if n > 0 {
			fmt.Print(string(buf[:n]))
		}
	}
}
