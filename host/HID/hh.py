import hid
import time
import numpy as np

# Replace with your device's vendor ID and product ID
VENDOR_ID = 0x0483
PRODUCT_ID = 0x5750

NUMBER_OF_PIXELS = 3648
NUM_VALUES_IN_PACKET = 31

def read_full_ccd_array(device):
    result = np.zeros(NUMBER_OF_PIXELS, dtype=np.float64)
    need_indices = {i for i in range((NUMBER_OF_PIXELS + NUM_VALUES_IN_PACKET - 1) // NUM_VALUES_IN_PACKET)}
    while (len(need_indices) > 0):
        #print(len(need_indices))
        data = device.read(64, timeout_ms=1000)
        if data:
            index = int(data[0])
            #print(index)
            if index in need_indices:
                need_indices.remove(index)
            repacked_data = np.array(data[2:]).reshape(-1, 2)
            uint16_values = repacked_data[:, 0].astype(np.uint16) | \
                   (repacked_data[:, 1].astype(np.uint16) << 8)
            slice_from = NUM_VALUES_IN_PACKET * index
            slice_to = min(NUM_VALUES_IN_PACKET * (index + 1), NUMBER_OF_PIXELS)
            result[slice_from:slice_to] = \
                uint16_values[:slice_to - slice_from]
        else:
            break

    return result

if __name__ == "__main__":
  device = hid.device()
  device.open(VENDOR_ID, PRODUCT_ID)
  print(f'Connected to device')
  val = read_full_ccd_array(device)
  print(val)
  print(val[300:350])
  print(val[2000:2050])
  print(val[3000:3050])
  device.close()

