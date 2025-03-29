# 1) First ensure you have the 'qrcode' library installed:
#    pip install qrcode[pil]

import qrcode

# 2) Create and save the QR code
data_to_encode = "https://192.168.0.1"
img = qrcode.make(data_to_encode)
img.save("qrcode_192.168.0.1.png")

print("QR code generated and saved to 'qrcode_192.168.0.1.png'.")
