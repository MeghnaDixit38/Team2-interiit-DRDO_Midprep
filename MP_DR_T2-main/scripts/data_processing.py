import numpy as np
from scipy.signal import savgol_filter

def process_data():
    arr = np.load('road_2.npy')
    l = arr
    temp = []
    for i in range(0, len(arr)):
        if len(temp) > 0:
            t = np.sqrt(np.sum(np.square(temp[-1] - l[i])))
            if t > 1:
                temp.append(l[i])
        else:
            temp.append(l[i])


    temp = np.array(temp)

    j = -1
    s = savgol_filter([temp[0:j,1], temp[0:j, 0]], 31, 3)

    w = 20
    new_ = []
    for i in range(0, int(len(temp)/w)):
        temp_ = [temp[w*i:w*(i + 1), 0], s[0][w*i:w*(i + 1)]]
        coff = np.polyfit(temp_[0], temp_[1], 3)
        p = np.poly1d(coff)
        temp_[1] = p(temp_[0])
        new_.append(p(temp_[0]))

    new_ = np.reshape(new_, (np.array(new_).shape[0]*np.array(new_).shape[1]))
    n= len(new_)
    final = [temp[:n,0], new_[:n], temp[:n, 2]]
    final = np.array(final)

    np.save('final.npy', final)
    return final


# if __name__ == '__main__':
#     main()
