#!/usr/bin/env python

#	Цветное изображение конвертируем спомощью файла IMG_to_HEX_converter.py
#	запускаем командную строку CMD
#   устанавливаем если нет модуль Image -> pip3 install image
#	кидаем фаил IMG_to_HEX_converter.py и картинку в одну папку
#	потом в командной строке переходим в эту папку и пишем:
#	> python   IMG_to_HEX_converter.py   (имя файла)logo.jpg    (ширина)85    (высота)80
#
#	например:    D:\IMG>python  IMG_to_HEX_converter.py  logo.jpg  85  80
#
#	и в этой же папке пояаиться фаил с нашим массивом picFile.txt
#
#	ГЕНЕРИРУЕТ В ФОРМАТЕ   RGB565 ( 16bit )


from PIL import Image
import sys
import os



if len(sys.argv) != 4:
    print("Usage: {} <image-file> <width> <height>".format(sys.argv[0]))
    sys.exit(1)

fname = sys.argv[1]

W = sys.argv[2]
print("\r\n")
print("<width> " + str(W))

H = sys.argv[3]
print("<height> " + str(H))

img = Image.open(fname)
if img.width != int(W) or img.height != int(H):
    print("Error: Разрешение указано не верно ( должно совпадать с картинкой )!!!");
    sys.exit(2)
	
f=open("picFile.txt", "a")

f.write("// size " + str(W) + " x " + str(H) + " \n");
f.write("const unsigned short img[] = { \n ");

for y in range(0, img.height):
    s = ""
    for x in range(0, img.width):
        (r, g, b) = img.getpixel( (x, y) )
        color565 = ((r & 0xF8) << 8) | ((g & 0xFC) << 3) | ((b & 0xF8) >> 3)
        # for right endiness, so ST7735_DrawImage would work
        color565 = ((color565 & 0xFF00) >> 8) | ((color565 & 0xFF) << 8)
        s += "0x{:04X},".format(color565)
    s += "  \n"
    f.write(s)

f.write("}; \r\n")
f.close()
print("\r\n Готово !!! OK \r\n")
