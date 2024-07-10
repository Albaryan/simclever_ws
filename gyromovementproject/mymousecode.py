from pynput import mouse
import time

# The event listener will be running in this block

countDistance=0

with mouse.Events() as events:
    # Block at most one second
    while True:
        event = events.get(None)
        if event is not None and event.__class__.__name__=='Click' and event.pressed==False:
            if countDistance:
                countDistance=0
                print("Calculation finished")
                displacementx-=event.x
                displacementy-=event.y
                displacement=(displacementx**2 + displacementy**2)**0.5
                print(f"Moved cursor by {dcm} cm on rl and {displacement} pixels on pc")
            else:
                countDistance=1
                print("Calculation started")
                displacementx=event.x
                displacementy=event.y
                dcm=input("cm for mouse movement :")
            

            
                
        time.sleep(0)