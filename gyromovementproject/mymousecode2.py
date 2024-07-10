from pynput import mouse
import time
import cv2
import numpy as np

PIXEL_TO_METER=402.659375
countDistance=0 # Mesafe sayılabilir mi?

img = np.zeros((900,900),dtype=np.uint8) # Boş görüntü oluştur

img[450,450]=255 # Görüntünün merkezini beyaz kare yap

center=[450,450] # Görüntünün merkezi

movement=None # Fare hareketinin tutulacağı değişken


with mouse.Events() as events:
    while True:
        event = events.get(0.0) #0.0 saniye boyunca fare etkinliği bekle
        
        if event is not None and event.__class__.__name__=='Move': #Fareden gelen etkinlik var mı ve bu etkinlik hareket etkinliği mi?
            
            if countDistance: # Mesafe sayımı serbest mi?
                movement=[event.x,event.y] # Gidilen mesafeyi kaydet
            else: # Mesafe sayımı serbest değilse (tek seferlik çalışıp başlangıç konumunu alır)
                countDistance=1 # Mesafe sayımı serbest
                displacementx=event.x #X ilk değer
                displacementy=event.y #Y ilk değer
                
        else:
            
            
            if movement!=None: # Movement değişkeni tanımsız değilse
                displacementx=movement[0]-displacementx #Yer değiştirmeyi hesapla
                displacementy=movement[1]-displacementy #
            
                displacement=(displacementx**2 + displacementy**2)**0.5 # Delta yer değiştirmeyi hesapla
                
                newCenter = [center[0]+round(displacementx/PIXEL_TO_METER*100),round(center[1]+(displacementy/PIXEL_TO_METER*100))] # Yeni çizgi merkezi
                cv2.line(img,center,newCenter,(255,255,255),1) # Eski çizgi merkezinden yeni çizgi merkezine çizgi çiz
                center=newCenter # Eski merkezi yeni merkeze eşitle
                #cv2.putText(img,f"{displacement/PIXEL_TO_METER-(displacement/PIXEL_TO_METER%0.01)}cm",center,cv2.FONT_HERSHEY_SIMPLEX,1,(255,255,255),thickness=1,lineType=cv2.LINE_AA)
                print(f"{displacement/PIXEL_TO_METER-(displacement/PIXEL_TO_METER%0.01)}cm") # Kaç cm yer değiştirildiğini yaz
                movement=None # Movement değişkenini tanımsız yap
                countDistance=0 # Mesafe saymak müsait değil
                
        cv2.imshow("img",img) #Ekranda göster
        cv2.waitKey(1)        #
                
        time.sleep(0) #Keyboard Interrupt (ctrl+c) için 0 saniyelik bekleme