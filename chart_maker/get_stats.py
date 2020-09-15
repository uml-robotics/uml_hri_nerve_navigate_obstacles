#!/usr/bin/python3
#
# get_stats.py
#
# simple utility to convert a csv into summary stats + charts
# for ros navigation testing

import matplotlib.pyplot as plt
import numpy as np
import pandas as pd
from PIL import Image
from sys import argv

# read csv data into dataframe object
df = pd.read_csv("pioneer_logger.csv")

# replace collision's null values w/ 0
df.replace({'field.collision.x':-100000, 'field.collision.y':-100000},0,inplace=True)

#remove unnecessary cols
df=df.drop(['field.header.frame_id'],axis=1)

# create new df for plotting robot's path
df_pos = df[['field.robot_pos.x','field.robot_pos.y']]
df_pos = df_pos.rename(columns={'field.robot_pos.x':'robot_pos.x','field.robot_pos.y':'robot_pos.y'})


#### create excel file w/ summary stats and raw data

# write to .xlsx file
with pd.ExcelWriter('robot_pos2_logger.xlsx') as writer:
    df.describe().to_excel(writer, sheet_name='Summary_Stats')
    df.to_excel(writer, sheet_name='Raw_Data')


#### build some charts

# plot robot's path
path = df_pos.plot.scatter(x='robot_pos.x',
                           y='robot_pos.y',
                           xlim=(-1,8),
                           ylim=(-2.0,3.00),
                           figsize=(9,7),
                           s=10,
                           c='cornflowerblue')
path.get_figure().savefig("robot_pos2.png")

# remove axis and save for superimposing on map
path.axis('off')
path.get_figure().savefig("robot_pos2_no_axis.png", transparent=True)

# load map and path_no_axis images for merging
# map is background, path_no_axis is foreground
background = Image.open("level1.pgm")
foreground = Image.open("robot_pos2_no_axis.png")

#crop background map
(left, upper, right, lower) = (175, 120, 300, 230)
background_c = background.crop((left, upper, right, lower))
#resize background map
background_cr = background_c.resize((int(foreground.width*1.2),int(foreground.height*1.2)))

# paste path_no_axis on map
background_cr.paste(foreground, (60,80), foreground.convert('RGBA'))

#save the final image
background_cr.save('robot_path_map_super.png')

