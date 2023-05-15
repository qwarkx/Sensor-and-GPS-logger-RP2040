
import numpy as np
import re

def removeDC(y):
    if type(y) is dict:
        try:
            buffer = None
            for key in y.keys():
                buffer[key] = y[key] - np.mean(y[key])
        except Exception as e:
            print(e)
        return buffer
    else:
        return y - np.median(y)


def signal_max_min(data, graphSize):
    max_ofset = []
    min_ofset = []
    mid_set = []
    for i in range(0, graphSize):
        max_ofset.append(abs(max(data[i])))
        mid_set.append(abs(np.median(data[i])))
        min_ofset.append(abs(min(data[i])))
    return max_ofset, min_ofset, mid_set

def y_offset(y, offset, graphs, shadow_en = 0):
    ## offset[offsetall, offsetby3, offsetby2group3by3 ]
    offset_number = 0
    offset_calc = 0
    offset_calc_out = []
    y_data = []
    shadow_y_data = []
    y_data_arnyek = np.zeros([len(y),len(y[0])])

    max_of, min_of, mid_of = signal_max_min(y,graphs)
    data_range = range(1, graphs +1)

    if offset is 'offsetall':
        offset_number = 1
        for j in range(0, graphs):
            offset_calc = offset_calc + (max_of[j] + min_of[j - 1] + pow(10, 3)) * offset_number
            offset_calc_out.append(offset_calc)
            #y_data.insert(j, y[j] + np.array(offset_calc * -1))


    if 'offsetby' in offset and 'group' not in offset:
        offset_number = float(offset.split("by")[1])
        for j in range(0, graphs):
            offset_calc = offset_calc + (max_of[j] + min_of[j - 1] + pow(10, 3)) * offset_number
            offset_calc_out.append(offset_calc)
            #y_data.insert(j, y[j] + np.array(offset_calc * -1))

    if 'offsetby' in offset and 'group' in offset:
        #usage > offsetby2group3by3
        #print('Length of the data %d' % len(y))
        settings = re.findall(r'\d+', offset)
        offset_buff = float(settings[0])
        group_number = float(settings[1])
        group_space = float(settings[2])
        for j in range(0, graphs):
            if (j % group_number == 0) and j != 0 and (graphs % group_number == 0):
                offset_number = group_space
            else:
                offset_number = offset_buff
            offset_calc = offset_calc + (max_of[j] + min_of[j - 1] + pow(10, 3)) * offset_number
            offset_calc_out.append(offset_calc)
            #y_data.insert(j, y[j] + np.array(offset_calc * -1))

    if shadow_en:
        for j in range(0, graphs):
            y_data.insert(j, y[j] + np.array(offset_calc_out[j] * -1))
            shadow_y_data.insert(j, y_data_arnyek[j] + np.array(offset_calc_out[j] * -1))
        return y_data, shadow_y_data
    else:
        y_data.insert(j, y[j] + np.array(offset_calc_out[j] * -1))
        return y_data

    return y_data




def y_offsetdddd(y, offset, graphs, zeropoint_en = 0):
    ## offset[offset_auto, offsetby3, offsetby2group3by3 ]

    offset_number = 0
    offset_calc = 0
    y_data = []
    y_data_arnyek = np.zeros([len(y),len(y[0])])

    max_of, min_of, mid_of = signal_max_min(y,graphs)
    # data_range = range(1, graphs +1)

    if 'offset_auto' in offset:
        offset_calc_out = []

        try: offset_number = float(re.findall(r'\d+', offset)[0])
        except: offset_number = 2

        for j in range(0, graphs):
            offset_calc = offset_calc + (max_of[j] + min_of[j - 1]) * offset_number
            offset_calc_out.append(offset_calc)

    if offset is 'offset_auto_group':
        group_number = 3
        group_space = 2
        offset_calc_out = []

        try: offset_buff = float(re.findall(r'\d+', offset)[0])
        except: offset_buff = 1

        for j in range(0, graphs):
            if (j % group_number == 0) and j != 0 and (graphs % group_number == 0):
                offset_number = group_space
            else:
                offset_number = offset_buff

            offset_calc = offset_calc + ((max_of[j] + min_of[j - 1]) * offset_number)
            offset_calc_out.append(offset_calc)

    if zeropoint_en:
        zeropoint_y_data = []

        for j in range(0, graphs):
            y_data.insert(j, y[j] + np.array(offset_calc_out[j] * -1))
            zeropoint_y_data.insert(j, y_data_arnyek[j] + np.array(offset_calc_out[j] * -1))
        return y_data, zeropoint_y_data
    else:
        y_data.insert(j, y[j] + np.array(offset_calc_out[j] * -1))
        return y_data

    return y_data