package main

import (
	"bufio"
	"fmt"
	"go.bug.st/serial"
	"log"
	"encoding/json"
)

const SerialPort = "/dev/tty.usbserial-110"

type SensorData struct {
	R_AC float64 `json:"r_ac"`
	R_DC float64 `json:"r_dc"`
	G_AC float64 `json:"g_ac"`
	G_DC float64 `json:"g_dc"`
	B_AC float64 `json:"b_ac"`
	B_DC float64 `json:"b_dc"`
}


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

	// buf := make([]byte, 512)
	// for {
	// 	n, err := port.Read(buf)
	// 	if err != nil {
	// 		log.Fatal(err)
	// 	}
	// 	if n > 0 {
	// 		fmt.Print(string(buf[:n]))
	// 	}
	// }

	scanner := bufio.NewScanner(port)
	fmt.Println("Listening for sensor data...")

	for scanner.Scan() {
		line := scanner.Text()
		var data SensorData

		if err := json.Unmarshal([]byte(line), &data); err != nil {
			fmt.Printf("Error parsing JSON: %v\nLine: %s\n", err, line)
			continue
		}

		// compute RoR

		var ror, ratioRed, ratioGreen float64

		// ratio_red = r_ac / r_dc if r_dc != 0 else 0
		// ratio_green = g_ac / g_dc if g_dc != 0 else 0
		// ror = ratio_green / ratio_red if ratio_red != 0 else 0

		if data.R_DC != 0 {
			ratioRed = data.R_AC / data.R_DC
		} else {
			ratioRed = 0
		}

		if data.G_DC != 0 {
			ratioGreen = data.G_AC / data.G_DC
		} else {
			ratioGreen = 0
		}
		
		if ratioRed != 0 {
			ror = ratioGreen / ratioRed
		} else {
			ror = 0
		}

		// Print structured sensor data
		fmt.Printf("RoR: %f\n", ror)
	}

	if err := scanner.Err(); err != nil {
		log.Fatal(err)
	}
}
