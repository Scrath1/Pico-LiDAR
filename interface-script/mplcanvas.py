import matplotlib
matplotlib.use('QtAgg')
from matplotlib.backends.backend_qt5agg import FigureCanvasQTAgg
from matplotlib.figure import Figure

class MplCanvas(FigureCanvasQTAgg):
    def __init__(self, parent=None, width=5, height=4, dpi=100):
        self.fig = Figure(figsize=(width, height), dpi=dpi)
        super().__init__(self.fig)
    
    # def set_plot_setup_func(self, func):
    #     self._plot_setup_function = func
        
    # def set_plot_func(self, func):
    #     self._plot_function = func
        
    # def plot(self, x, y):
    #     self._plot_function(x, y)