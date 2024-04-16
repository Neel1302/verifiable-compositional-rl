import math

def minigrid2airsim(obs):
    '''
    Minigrid (x,y,yaw) -> AirSim (N,E,yaw) coordinates
    '''
    x_minigrid, y_minigrid, yaw_minigrid = obs
    x_cells, y_cells = [25,25]
    x_border, y_border = [2,2]
    x_max, y_max = [128,128]
    
    n_airsim = y_max-(y_minigrid-y_border)*(2*y_max)/(y_cells-2*y_border-1)
    e_airsim = (x_minigrid-x_border)*(2*x_max)/(x_cells-2*x_border-1)-x_max
    
    # Yaw is not yet supported in ROS API
    yaw_airsim = yaw_minigrid
    return ([n_airsim, e_airsim, yaw_airsim])

# def airsim2minigrid(obs):
#     '''
#     AirSim (N,E,yaw) -> Minigrid (x,y,yaw) coordinates
#     '''
#     n_airsim, e_airsim, yaw_airsim = obs
#     x_cells, y_cells = [25,25]
#     x_border, y_border = [2,2]
#     x_max, y_max = [128,128]
    
#     x_minigrid = x_border+(e_airsim+x_max)*(x_cells-2*x_border-1)/(2*x_max)
#     y_minigrid = y_border-(n_airsim-y_max)*(y_cells-2*y_border-1)/(2*y_max)
    
#     print(x_minigrid, y_minigrid)
#     x_minigrid, y_minigrid = (math.ceil(x_minigrid), math.ceil(y_minigrid))
    
#     # Yaw is not yet supported in ROS AP
#     yaw_minigrid = yaw_airsim
    
#     return ([x_minigrid, y_minigrid, yaw_minigrid])

def airsim2minigrid(obs):
    '''
    AirSim (N,E,yaw) -> Minigrid (x,y,yaw) coordinates
    '''
    n_airsim, e_airsim, yaw_airsim = obs
    x_cells, y_cells = [25,25]
    x_border, y_border = [2,2]
    x_max, y_max = [128,128]
    
    cell_size = (2*x_max)/(x_cells-2*x_border-1)
    
    range_list = []
    for i in range(-12,13):
        range_list.append([i*cell_size-cell_size/2, i*cell_size+cell_size/2])
    
    for i, range_ in enumerate(range_list):
        if ((n_airsim > range_[0]) and (n_airsim < range_[1])):
            y_minigrid = i
        if ((e_airsim > range_[0]) and (e_airsim < range_[1])):
            x_minigrid = i
    
    # print(x_minigrid, y_minigrid)
    
    # Yaw is not yet supported in ROS AP
    yaw_minigrid = yaw_airsim
    
    return ([x_minigrid, y_minigrid, yaw_minigrid])