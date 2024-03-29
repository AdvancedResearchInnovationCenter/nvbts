from sklearn.cluster import KMeans
import numpy as np
import matplotlib.pyplot as plt

def sort_markers_k_means(markers, k=8):
    """
    Sorts the markers in the order of the layers using k-means clustering.
    The markers are first clustered into k clusters using k-means clustering.
    Then, the clusters are sorted by their mean radius and the markers in each
    cluster are sorted by their angle.

    Parameters
    ----------
    markers : np.ndarray of shape (n, 2)
    k : int
        Number of clusters = number of layers - 1.
    
    Returns
    -------
    x_coord : list
        List of x-coordinates of the sorted markers.
    y_coord : list
        List of y-coordinates of the sorted markers.
    layer : list
        Layer of each marker.
    """

    markers_x = markers[:, 0]
    markers_y = markers[:, 1]
    marker_layers = []

    center_x = np.median(markers_x)
    center_y = np.median(markers_y)

    closest = np.argmin(np.sqrt((markers_x - center_x)**2 + (markers_y - center_y)**2))

    center_x, center_y = markers_x[closest], markers_y[closest]
    marker_layers.append(np.array([[center_x, center_y, 0, 0]]))

    #drop the center
    markers_xx = np.delete(markers_x, closest)
    markers_yy = np.delete(markers_y, closest)


    x = np.array(markers_xx) - center_x
    y = np.array(markers_yy) - center_y
    r = np.sqrt(x**2 + y**2)
    tan2_phi = np.arctan2(y, x)
    tan2_phi[tan2_phi < 0] += 2 * np.pi
    tan2_phi[tan2_phi > 2*np.pi] -= 2 * np.pi

    markers = np.array([markers_xx, markers_yy, r, tan2_phi]).T

    kmeans = KMeans(n_clusters=k, algorithm='lloyd', n_init='auto')

    kmeans.fit(r.reshape(-1, 1))
    labels = kmeans.labels_
    sorted_indices = np.argsort(kmeans.cluster_centers_.flatten())

    for i in range(k):
        marker_layers.append(np.array(markers[labels == sorted_indices[i]]))
        plt.scatter(marker_layers[i+1][:, 0], marker_layers[i+1][:, 1])

    x_coord = []
    y_coord = []
    la = []
    for layer, i in zip(marker_layers, range(len(marker_layers))):
        if i == 0:
            x_coord.append(layer[:,0].item())
            y_coord.append(layer[:,1].item())
            la.append(0)
        else:
            cluster_center = [np.median(layer[:,0]), np.median(layer[:,1])]
            tan1_vec = np.arctan2(layer[:,1]-cluster_center[1], layer[:,0]-cluster_center[0])
            right_marker = np.argmin(np.abs(tan1_vec))
            x_coord.append(layer[right_marker,0].tolist())
            y_coord.append(layer[right_marker,1].tolist())
            la.append(i)
            layer_tmp = np.delete(layer, right_marker, axis=0)
            sorted_layer_indices = np.argsort(layer_tmp[:,3].flatten())
            sorted_layer_indices = sorted_layer_indices[::-1]
            for j in sorted_layer_indices:
                x_tmp = layer_tmp[j,0]
                y_tmp = layer_tmp[j,1]
                x_coord.append(x_tmp)
                y_coord.append(y_tmp)
                la.append(i)

    return np.stack((x_coord, y_coord), axis=-1), la
