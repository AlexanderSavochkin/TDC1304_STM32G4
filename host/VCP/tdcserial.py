import serial
import struct

def read_adc_stream(port, baudrate=115200, magic_header=b'\xfa\xba', samples=3648):
    """
    Generator that continuously reads 12-bit ADC values from STM32 VCP.
    Yields lists of 3648 values when valid frames are found.
    """
    ser = serial.Serial(port, baudrate, timeout=0.1)
    buffer = bytearray()
    payload_size = samples * 2  # 2 bytes per sample (16-bit container for 12-bit ADC)

    try:
        while True:
            # Read available bytes and extend buffer
            buffer.extend(ser.read(ser.in_waiting or 1))
            print(len(buffer))

            # Continuously scan buffer for complete frames
            while True:
                # Find magic header in buffer
                header_pos = buffer.find(magic_header)
                if header_pos == -1:
                    break  # No header found yet

                # Calculate payload start/end positions
                payload_start = header_pos + len(magic_header)
                payload_end = payload_start + payload_size

                if len(buffer) < payload_end:
                    # Not enough data yet; wait for more and continue filling the buffer
                    break

                # Extract and process payload
                payload = buffer[payload_start:payload_end]
                del buffer[:payload_end]  # Remove processed data

                # Unpack 16-bit values and convert to 12-bit (mask 0xFFF)
                values = struct.unpack(f'<{samples}H', payload)
                yield [v & 0xFFF for v in values]  # Mask to 12 bits

    finally:
        ser.close()




def read_one_buffer(port, baudrate=115200, magic_header=b'\xfa\xba', samples=3648):
    """
    Reads a single buffer (frame) of ADC data from the read_adc_stream generator.

    Args:
        port (str): The serial port to connect to (e.g., '/dev/ttyACM0').
        baudrate (int): The baud rate for the serial connection (default: 115200).
        magic_header (bytes): The magic number marking the start of a frame (default: b'\xba\xfa').
        samples (int): The number of 16-bit samples in a frame (default: 3648).

    Returns:
        list[int]: A list of 12-bit ADC values from the received frame.
    """
    # Create the generator
    adc_stream = read_adc_stream(port, baudrate, magic_header, samples)

    # Read and return one frame
    try:
        return next(adc_stream)
    except StopIteration:
        raise RuntimeError("No data available from the ADC stream.")



# Usage example
if __name__ == "__main__":
    PORT = '/dev/ttyACM0'  # Update with your virtual COM port

#    for adc_frame in read_adc_stream(PORT):
#        print(f"Received frame with first 5 values: {adc_frame[:5]}")
#        # Add your processing logic here

    tcd_frame = read_one_buffer(PORT)
    print(f"Received data: {tcd_frame}")
