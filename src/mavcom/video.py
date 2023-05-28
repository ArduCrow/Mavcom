import threading
import cv2
import time
import os
import socket

class Recorder():
    """
    Object that controls recording of onboard video. 
    
    Parameters
    ----------
    
    - `controller`: This should be the upstream class that instantiates the Recorder instance. By passing itself to
    the Recorder instance, the Recorder can access its attributes.
    
    - `top_text`: text that will be overlaid on this position of the video capture.
    
    - `centre_text`: text that will be overlaid on this position of the video capture.
    
    - `bottom_text`: text that will be overlaid on this position of the video capture.
    
    - `filename`: OPTIONAL - the desired filename. If none is set, it will create a folder 'recordings'
    in the directory the program was started in and save recordings as `[hostname]-n` where 'n' is the n number of recordings in
    the folder plus 1.
    
    - `show_video`: boolean True or False, only really for testing; shows the video output on-screen.
    """
    
    def __init__(self, controller: object = None, top_text = None, centre_text = None, bottom_text = None, filename=None, show_video=False) -> None:
        self.top_text = top_text
        self.centre_text = centre_text
        self.bottom_text = bottom_text
        self.video_active = False
        self.show = show_video
        self.controller = controller
        
        
        self.recording = threading.Thread(target=self.record, daemon=True)
        self.hud_updates = threading.Thread(target=self.hud_updater, daemon=True)
        
        os.makedirs("recordings", exist_ok=True)
        if filename is None:
            self.filename = f"{socket.gethostname()}-{len(os.listdir('recordings'))}"
        print(self.filename)
        
    def record(self) -> None:
        """
        Start recording video. Start by calling `start()` on recording thread.
        """
        self.video_active = True
        font_scale = 2
        font = cv2.FONT_HERSHEY_PLAIN

        camera = cv2.VideoCapture(-1)
        width = camera.get(3)
        height = camera.get(4)
        fps = camera.get(5)

        fourcc = cv2.VideoWriter_fourcc(*'XVID')
        output = cv2.VideoWriter(f"recordings/{self.filename}.mp4", fourcc, 20, (int(width), int(height)))
        time.sleep(1)
            
        print("RECORDER: video active")
        while self.video_active:
            ret, frame = camera.read()
            
            # use these to flip your image if required
            # frame = cv2.flip(frame, flipCode=0)
            # frame = cv2.flip(frame, flipCode=1)
            
            cv2.putText(frame, f"{self.top_text}",(20, 50), font, fontScale=font_scale, color=(0,255,0), thickness=2)
            if self.centre_text:
                cv2.putText(frame, f"{self.centre_text}",(int(width/2)-70, int(height/2)), font, fontScale=font_scale, color=(0,255,0), thickness=2)
            if self.bottom_text:
                cv2.putText(frame, f"{self.bottom_text}",(int(width/2)-180, int(height/2)+180), font, fontScale=font_scale, color=(0,255,0), thickness=2)
            if self.show:
                cv2.imshow("Feed", frame)
            output.write(frame)
            if cv2.waitKey(5) & 0xFF == ord("c"):
                break
            
        camera.release()
        output.release()
        print("RECORDER: end of video")
        
    def video_mod(self, text_pos: str, text_content: str) -> None:
        """
        Edit the text overlay of the video.
        
        Parameters
        ----------
        
        - `text_pos`: the position of the text to modify/overlay. Options:
        
            - `top_text`
            
            - `centre_text`
            - `bottom_text`
        
        - `text_content`: the content to put into this position
        """
        setattr(self, text_pos, text_content)
        
    def hud_updater(self) -> None:
        """
        Continually updates the top text overlay of the video with name, alt and speed information.
        Start by calling `start()` on hud_updates thread.
        """
        while self.video_active:
            try:
                alt = self.controller.vehicle_state.alt
                speed = self.controller.vehicle_state.groundspeed
                self.video_mod("top_text", f"{socket.gethostname()} Alt: {alt} Speed: {speed}")
            except Exception as e:
                self.video_mod("top_text", f"{socket.gethostname()}")
                pass
            time.sleep(0.5)
