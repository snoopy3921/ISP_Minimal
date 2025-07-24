from PIL import Image

# Load and resize image
img = Image.open('ISP_minimal/flower.jpg').convert('RGB')
img = img.resize((576, 576))

with open('ISP_minimal/new_test_rgb565.frm', 'wb') as f:
    for y in range(576):
        for x in range(576):
            r, g, b = img.getpixel((x, y))
            # Convert to RGB565
            rgb565 = ((r & 0xF8) << 8) | ((g & 0xFC) << 3) | (b >> 3)
            f.write(rgb565.to_bytes(2, byteorder='big'))