import cv2
import numpy as np
import colorsys  

class Draw:

    def __init__(self, frame, trackers, fps, linewidth=2, height=720, width=1280, mode='tracker'):
        """
        Class to draw tracker on target image and create necessary visualization
        """
        self.frame=frame
        self.trackers=trackers
        self.linewidth=linewidth
        self.height=height
        self.width=width
        self.mode=mode#more modes to be implemented in the future
        self.fps=fps
    
    def create_color(self, tag, hue_step=.41):
        # print("tag: ", tag)
        h, v = (tag * hue_step) % 1, 1. - (int(tag * hue_step) % 4) / 5.
        r, g, b = colorsys.hsv_to_rgb(h, 1., v)
        return int(255*r), int(255*g), int(255*b)
    
    def bboxes(self):
        if self.mode=="tracker":
            for trk in self.trackers:
                
                draw = False
                if int(trk[5]) == 2:
                    class_="person"
                    draw = True
                elif int(trk[5]) == 4:
                    class_="car"
                    draw = True
                elif int(trk[5]) == 5:
                    class_ = "motorcycle"
                    draw = True
                # elif int(trk[5]) == 45:
                #     class_ = "bottle"
                # elif int(trk[5]) == 77:
                #     class_ = "keyboard"
                # else:
                #     class_ = "other"

                if draw:
                    cv2.rectangle(self.frame, (int(trk[0]), int(trk[1])), (int(trk[2]), int(trk[3])), self.create_color(trk[6]), self.linewidth)
                    text="{}:{:.1f},x:{:.2f},y:{:.2f}w:{:.2f},h:{:.2f}".format(class_, trk[4], trk[-4], trk[-3], trk[-2], trk[-1])
                    # cv2.putText(self.frame, "FPS:{}".format(int(self.fps)), org=(50, 70), fontFace=cv2.FONT_HERSHEY_SIMPLEX, 
                    #             fontScale=1, color=(55, 255, 255),  thickness=2)
                    cv2.putText(self.frame, text, (int(trk[0]), int(trk[1])-10), cv2.FONT_HERSHEY_COMPLEX , 0.5, self.create_color(trk[6]),  2)
        else:
            pass
        return self.frame 
