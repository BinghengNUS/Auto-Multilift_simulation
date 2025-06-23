uint8 idx                     # drone index
float32[] xi_temp            # [h+1, nxi] flatten state trajectory of ith drone
float32[] ui_temp            # [h, nui]   flatten control trajectory of ith drone
float32 max_viol_i           # max violation value of ith drone
