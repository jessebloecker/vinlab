#!/usr/bin/env python
import numpy as np
class ColorHelper():

    #rgb constants
    WHITE         = np.array([255,255,255], dtype=np.uint8)
    BLACK         = np.array([0,0,0], dtype=np.uint8)
    GREY          = np.array([128,128,128], dtype=np.uint8)
    DARK_GREY     = np.array([64,64,64], dtype=np.uint8)
    BROWN         = np.array([120,91,44], dtype=np.uint8)
    RED           = np.array([255,0,0], dtype=np.uint8)           
    DARK_RED      = np.array([128,0,0], dtype=np.uint8)
    RED_ORANGE    = np.array([255,50,0], dtype=np.uint8)
    ORANGE        = np.array([255,150,0], dtype=np.uint8)
    ORANGE_YELLOW = np.array([255,200,0], dtype=np.uint8)
    YELLOW        = np.array([255,255,0], dtype=np.uint8)
    YELLOW_GREEN  = np.array([128,255,0], dtype=np.uint8)
    GREEN         = np.array([0,255,0], dtype=np.uint8)
    DARK_GREEN    = np.array([0,128,0], dtype=np.uint8) 
    GREEN_BLUE    = np.array([0,255,128], dtype=np.uint8)     
    CYAN          = np.array([0,255,255], dtype=np.uint8) 
    BLUE          = np.array([0,0,255], dtype=np.uint8) 
    DARK_BLUE     = np.array([0,0,128], dtype=np.uint8)
    BLUE_PURPLE   = np.array([111,3,252], dtype=np.uint8)
    PURPLE        = np.array([160,32,240], dtype=np.uint8)
    DARK_PURPLE   = np.array([72,16,107], dtype=np.uint8)
    PINK          = np.array([255,0,255], dtype=np.uint8)
    GOLD          = np.array([145,129,0], dtype=np.uint8)

    def __init__(self, format='rgb', scale=255, dtype=None, alpha=None):
        self.set_defaults()
        if format == 'rgb' and scale == 255 and dtype == None:
            pass
        else:
            self.reformat(format, scale, dtype, alpha)

    def set_defaults(self):
        self.WHITE = ColorHelper.WHITE
        self.BLACK = ColorHelper.BLACK
        self.GREY = ColorHelper.GREY
        self.DARK_GREY = ColorHelper.DARK_GREY
        self.BROWN = ColorHelper.BROWN
        self.RED = ColorHelper.RED
        self.DARK_RED = ColorHelper.DARK_RED
        self.RED_ORANGE = ColorHelper.RED_ORANGE
        self.ORANGE = ColorHelper.ORANGE
        self.ORANGE_YELLOW = ColorHelper.ORANGE_YELLOW
        self.YELLOW = ColorHelper.YELLOW
        self.YELLOW_GREEN = ColorHelper.YELLOW_GREEN
        self.GREEN = ColorHelper.GREEN
        self.DARK_GREEN = ColorHelper.DARK_GREEN
        self.GREEN_BLUE = ColorHelper.GREEN_BLUE
        self.CYAN = ColorHelper.CYAN
        self.BLUE = ColorHelper.BLUE
        self.DARK_BLUE = ColorHelper.DARK_BLUE
        self.BLUE_PURPLE = ColorHelper.BLUE_PURPLE
        self.PURPLE = ColorHelper.PURPLE
        self.DARK_PURPLE = ColorHelper.DARK_PURPLE
        self.PINK = ColorHelper.PINK
        self.GOLD = ColorHelper.GOLD


        self.RAINBOW_12 = np.vstack((self.RED, self.RED_ORANGE, self.ORANGE, self.ORANGE_YELLOW, self.YELLOW, self.YELLOW_GREEN,
                                   self.GREEN, self.GREEN_BLUE, self.CYAN,self.BLUE, self.BLUE_PURPLE, self.PURPLE))
        self.RAINBOW_21 = np.vstack((self.RAINBOW_12, self.DARK_RED, self.DARK_GREEN, self.DARK_BLUE, self.DARK_PURPLE, self.PINK,
                                    self.GREY, self.DARK_GREY, self.BROWN, self.GOLD))
        
        self.RAINBOW = self.RAINBOW_12

    def reformat(self, format='rgb', scale=255,dtype=None, alpha=None):
        self.set_defaults()
        if dtype is None:
            dtype = np.uint8 if int(scale)==255 else np.float16
        if alpha is None:
            alpha = scale
        for k,v  in self.__dict__.items():
            # print(k,v)
            if v.ndim == 2:
                alpha = scale*np.ones((len(v),1)).reshape(-1,1)


            v_scaled = (v.astype(np.float16)*(scale/255.)).astype(dtype)
            if format == 'rgb':
                pass

            elif format == 'rgba':
                setattr(self, k, np.hstack((v_scaled,dtype(alpha))))

            elif format == 'bgr':
                setattr(self, k, np.flip(v_scaled))

            elif format == 'bgra':
                setattr(self, k, np.append(np.flip(v_scaled),dtype(alpha)))

            elif format == 'hsv':
                pass

    def rainbow_sequence(self,n):
        """
        return n x 3 array cycling through the colors in self.RAINBOW_21
        """
        out = np.zeros((n,3))
        for i in range(n):
            out[i,:] = self.RAINBOW_21[i%20,:]
        return out.astype(np.uint8)