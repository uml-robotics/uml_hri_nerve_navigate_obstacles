#!/usr/bin/python3

"""
 * File:   nist_nav_test_report.py
 * Author: Daniel Lynch
 * Task:   Generate summary statistics and charts for NIST Industrial
 * reporting.
 *
 * Usage: python3 nist_nav_test_report.py your_data.csv
"""
from sys import argv
import matplotlib
import matplotlib.pyplot as plt
import numpy as np
import pandas as pd
import pathlib
from datetime import datetime

matplotlib.style.use('ggplot')

# get date/time-stamp
x = datetime.now()
now = x.strftime("%c").replace("  "," ").replace(" ","_")
# => 'Tue_Nov_3_10:44:22_2020'

# modify name given .csv such as "sim_log.csv"
report_name = argv[1].replace(".csv","")
# => sim_log

this_report = report_name + "_" + now
# => "sim_log_Tue_Nov_3_10:44:22_2020"

# create dir for given report
dir_w_figs = "./{0}/figs".format(this_report)
pathlib.Path(dir_w_figs).mkdir(parents=True, exist_ok=True)

# read csv data into dataframe object
df = pd.read_csv(argv[1])

# replace collision's null values w/ 0
df.replace({'field.collision.x':-100000, 'field.collision.y':-100000},0,inplace=True)

#remove unnecessary cols
df=df.drop(['field.header.frame_id'],axis=1)

### create robot position chart

# create new df for plotting robot's path
df_pos = df[['field.robot_pos.x','field.robot_pos.y']]
df_pos = df_pos.rename(columns={'field.robot_pos.x':'robot_pos.x','field.robot_pos.y':'robot_pos.y'})

# plot robot's path
robot_path = df_pos.plot.scatter(x='robot_pos.x',
                           y='robot_pos.y',
                           #xlim=(-1,6.8),
                           #ylim=(-1.0,13.00),
                           figsize=(20,20),
                           s=1,
                           c='cornflowerblue')

# save
path_fig = robot_path.get_figure()
path_fig.savefig("{0}/robot_position_graph.png".format(dir_w_figs))
plt.close(path_fig)


### get highlights metrics: successes, attempts, collisions

# note that in the logger, 1 iteration means 2 routes: a->b & b->a
number_of_attempts = df['field.iteration'].unique().size #* 2 
num_successful_routes = df['field.event'][df['field.event'].str.contains('Goal was reached', na=False)].size
route_success_rate = (num_successful_routes / number_of_attempts)*100

# get # collisions
number_of_collisions = df['field.collision.x'].unique().size - 1

highlights = { 'Label': ['Route Attempts','Successful Routes', 'Route Success Rate', 'Collisions'], 'Value': [number_of_attempts,num_successful_routes,route_success_rate,number_of_collisions] }
highlights_df = pd.DataFrame(highlights)

### time metrics: time summary stats, max route time, min route time

# get summary stats of route times

#split rows into "Goal x registered... Goal was reached" chunks
dfsub = df[['field.header.stamp','field.iteration','field.event']]
df_goals = dfsub[dfsub['field.event'].str.contains('Goal', na=False)]
#df_goals['field.header.stamp'].diff()
df_goal_times = pd.to_datetime(df_goals['field.header.stamp'])
time_to_goal_deltas = pd.DataFrame(df_goal_times.diff())
#drop first row
time_to_goal_deltas = time_to_goal_deltas[time_to_goal_deltas > pd.Timedelta(seconds=2)].dropna()


# mean and std of route times
mu = time_to_goal_deltas['field.header.stamp'].mean().seconds
sigma = time_to_goal_deltas['field.header.stamp'].std().seconds

# plot histogram of route times
time_to_goal = time_to_goal_deltas['field.header.stamp'].astype(
    'timedelta64[s]').plot.hist( ) #xlim=(55,70), ylim=(0,40), figsize=(10,5)
# comment below to remove inserting mu,sigma in graph
plt.text(68,30, r"$\mu={0}$, $\sigma={1}$".format(mu,sigma),horizontalalignment='right', fontsize=16)
plt.title('Distribution of Time to Goal per Route (s)', fontsize=18)
plt.xlabel('Time to Goal per Route (s)', fontsize=15)
plt.ylabel('Number of Runs',fontsize=15)

# save
#time_to_goal.get_figure().savefig("{0}/time_to_goal_per_route_histogram.png".format(dir_w_figs))
plt.savefig("{0}/time_to_goal_per_route_histogram.png".format(dir_w_figs))

### create excel file w/ summary stats and raw data

# write to .xlsx file
with pd.ExcelWriter('./{0}/{0}.xlsx'.format(this_report)) as writer:
    highlights_df.to_excel(writer, sheet_name='Summary', index=False)
    time_to_goal_deltas.describe().to_excel(writer, sheet_name='Summary', startcol=3)
    df.to_excel(writer, sheet_name='Raw_Data', index=False)


print("Completed: {0}".format(this_report))