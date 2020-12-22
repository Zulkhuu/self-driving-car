import matplotlib.pyplot as plt
import numpy as np

def plt_imlist(images, fig_size=(30,15)):
    plt.figure()
    plt.rcParams["figure.figsize"] = fig_size
    fig, axs = plt.subplots(1, len(images))
    for i in range(len(images)):
        if len(images[i].shape) == 2:
            # Binary image
            axs[i].imshow(np.dstack((images[i], images[i], images[i]))*255)
        else:
            axs[i].imshow(images[i])