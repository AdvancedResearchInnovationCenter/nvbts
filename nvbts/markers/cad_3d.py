import numpy as np


def rtpairs(r, n):        # Distribute the markers on a circle
    for i in range(len(r)):
       for j in range(n[i]):    
        yield r[i], j*(2 * np.pi / n[i])


def dome_3d(
        N = [1, 6, 12, 18, 24, 30, 36, 42],
        Dia = [0.0,  5.0, 10.0, 15.0, 20.0, 25.0, 30.0, 35.0],  
        S = [ 0.5, 0.5, 0.5, 0.5, 0.5, 0.5, 0.5, 0.5],
        H = 20,
        B = 40.0,
        T = 4,
    ):
    """Create a dome shaped 3D markers.

    Args:
        N (list, optional): Number of markers. Defaults to [1, 6, 12, 18, 24, 30, 36, 42].
        Dia (list, optional): Markers' circular pattern Diamters in mm. Defaults to [0.0,  5.0, 10.0, 15.0, 20.0, 25.0, 30.0, 35.0].
        S (list, optional): Markers' radius (markers' size). Defaults to 0.5 for each.
        H (int, optional): Sensor Height. Defaults to 20 mm.
        B (float, optional): Sensor Base Diamter. Defaults to 40.0 mm.
        T (int, optional): Sensor thickness. Defaults to 4 mm.
    """
    R = []    # Markers radii

    for i in Dia:
        R.append(i/2)    # Markers radii  = Dia / 2

    b = B /2       # Sensor Base radius
    h=(-b**2+H**2)/(2*H)


    Plane_Height = []      
    for r in R[1:]:
        z =  ((H-h)**2-r**2)**0.5 + h
        Plane_Height.append((z))

    markers_list = []      
    for r, t in rtpairs(R, N):
        x =  r * np.cos(t)/1000
        y =  r * np.sin(t)/1000
        z =  (((H-h)**2-r**2)**0.5 + h )/1000
        s = S[R.index(r)]/10000  # sensor size 
        
        markers_list.append((x,y,z))

    return np.array(markers_list)


def gelsight_mini_3d():
    x_m = np.linspace(0,8*1.5,9)
    y_m = np.linspace(0,6*1.5,7)
    z_m = np.zeros((1, 1))
    
    markers_Fs = np.stack(np.meshgrid(x_m, y_m, z_m))[:, :, :, 0].transpose((1, 2, 0))

    return markers_Fs.reshape(-1, 3)