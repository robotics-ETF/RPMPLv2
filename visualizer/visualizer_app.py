from kivy.config import Config
 
Config.set('graphics', 'resizable', 0)
Config.set('graphics', 'width', '1280')
Config.set('graphics', 'height', '800')

import kivy
from kivy.lang import Builder
from kivy.app import App
from kivy.uix.widget import Widget
from kivy.uix.floatlayout import FloatLayout
from kivy.graphics.vertex_instructions import Rectangle
from kivy.graphics.context_instructions import Color

kivy.require('2.0.0')

class VisualizationCanvas(Widget):

    def __init__(self,**kwargs):
        super(VisualizationCanvas,self).__init__(**kwargs)
        with self.canvas:
            Color(1, 0, 0, 1)
            Rectangle(pos=(300,500),size=self.size)

class AppRoot(FloatLayout):
    def __init__(self, **kwargs):
        super(AppRoot,self).__init__(**kwargs)
        with self.canvas:
            Color(1, 0, 0, 1)
            Rectangle(pos=self.pos,size=self.size)


class VisualizerApp(App):
    def build(self):
        return AppRoot()

if __name__ == '__main__':
    VisualizerApp().run()