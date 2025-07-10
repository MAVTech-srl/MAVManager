# Import packages
from dash import Dash, html, dash_table, dcc, callback, Output, Input, State, DiskcacheManager, set_props
import dash_daq as daq
import plotly.express as px
import subprocess
import numpy as np
import time
import os
import signal
import psutil

# Initialize the app
external_stylesheets = ['/home/davide/Documents/open3d_devel/dash_assets/style.css']
# app = Dash(external_stylesheets=external_stylesheets, assets_folder="assets")
app = Dash()

import diskcache
cache = diskcache.Cache("./cache")
background_callback_manager = DiskcacheManager(cache)

'''
Build top banner
'''
def build_banner():

    return html.Div(className="flex-container", children=[
        # html.Div(),
        # html.Div(),
        html.Div(className="flex-item", children=[
            html.Div(
                id="banner-text",
                children=[
                    html.H4("MAVManager"),
                    # html.H6("Drone monitoring and logging"),
                    html.Button(className="banner button",
                        id="help-button", children="HELP", n_clicks=0
                    )
                ],
            )
        ]),
        html.Div(className="flex-item", children=[
            html.A(html.Img(id="logo", src=app.get_asset_url("mavtech.png"), style={"height": "50px", "margin-right": "10px"}),
                    href="https://www.mavtech.eu/it/",
                    ),
        ]),
        # html.Div(className="flex-item"),
        # html.Div(),
        # html.Div(className="flex-item")
    ])


# Check the tty device names
bash_command = "ls -v /dev/ | awk '/^(ttyA|ttyU)/'" # List all files that begin with ttyA or ttyU
subproc = subprocess.run(bash_command, shell=True, check=True, executable='/bin/bash', stdout=subprocess.PIPE)
subproc_stdout = str(subproc.stdout.decode()).split()
# App layout
app.layout = [
    build_banner(),
    dcc.Tabs(className="custom tabs", children=[
        dcc.Tab(label='Control', children=[   
            html.Div(className="six columns", children=[
                html.Div(className="row", style={"padding": "10px"}, children=[
                    html.Br(),
                    dcc.Markdown(''' 
                                Select which UART port is used to communicate with the autopilot
                                '''),
                    dcc.Dropdown(subproc_stdout, id='tty-dropdown', placeholder='Select UART port...', clearable=True),
                ]),
                html.Div(className="row", id="lidar-model-selector", style={"padding": "10px"},  children=[
                    html.Br(),
                    dcc.Markdown('''Select LiDAR model'''),
                    dcc.Dropdown(['AVIA', 'MID360'], id='lidar-model-dropdown', placeholder='Select model...'),
                ]),
                
                html.Div(className="row", hidden=True, id="return-mode-selector", style={"padding": "10px"}, children=[
                    html.Br(),
                    dcc.Markdown('''Select LiDAR return mode'''),
                    dcc.Dropdown(['First single', 'Strongest single', 'Dual'], id='return-mode-dropdown', placeholder='Select return mode...'),
                ]),
                html.Div(className="row", hidden=True, id="scan-pattern-selector", style={"padding": "10px"},  children=[
                    html.Br(),
                    dcc.Markdown('''Select LiDAR scan pattern'''),
                    dcc.Dropdown(['Repetitive', 'Non-repetitive'], id='scan-pattern-dropdown', placeholder='Select scan pattern...'),
                ]),
                html.Br(),
                html.Div(className="two columns", id='slam-starter-div', style={"padding": "10px"},  children=[
                    dcc.ConfirmDialogProvider(
                        children=html.Button(className='button hover', children=['Start SLAM'], style={'color': 'white', 'background': 'green', 'text-align': 'center'}),
                        id='slam-starter',
                        message='The UAV will start the SLAM process. Continue?'
                    ),
                ]),
                html.Div(className="two columns", hidden=True, id='slam-stopper-div', style={"padding": "10px"},  children=[
                    dcc.ConfirmDialogProvider(
                        children=html.Button(className='button hover', children=['Stop SLAM'], style={'color': 'white', 'background': 'red'}),
                        id='slam-stopper',
                        message='Do you want to stop the SLAM process?'
                    ), 
                ]),
                html.Div(id='button-div'),
                html.Div(id="dummy-div")
            ]),
            html.Div(className="six columns", children=[
                html.Br(),
                html.Div(className="row", style={"padding": "10px"}, children=[
                    dcc.Markdown(''' 
                                Status
                                '''),
                    dcc.Textarea(id="status-terminal", readOnly=True, style={'width': '100%', 'height': 200})
                ])
            ]),
        ]),
        dcc.Tab(label='Logs', children=[
            html.Div(className="six columns", children=[
                html.Br(),
                html.Div(className="row", style={"padding": "10px"}, children=[
                    dcc.Markdown("SLAM logs"),
                    dcc.Textarea(id="slam-terminal", readOnly=True, style={'width': '100%', 
                                                                            'height': 200}),
                    html.Div(id="slam-pid")
                ]),
                #
                html.Br(),
                html.Div(className="row", style={"padding": "10px"}, children=[
                    dcc.Markdown("MAVROS logs"),
                    dcc.Textarea(id="mavros-terminal", readOnly=True, style={'width': '100%', 'height': 200}),
                    html.Div(id="mavros-pid")
                ]),
            ]),
            html.Div(className="six columns", children=[
                html.Br(),
                html.Div(className="row", style={"padding": "10px"}, children=[
                    dcc.Markdown("PTP logs"),
                    dcc.Textarea(id="ptp-terminal", readOnly=True, style={'width': '100%', 'height': 200}),
                    html.Div(id="ptp-pid")
                ])
            ]),
        ])
    ])

]

###############################################################
########################## CALLBACKS ##########################
###############################################################

# Select if AVIA or MID360
@callback(Output(component_id='return-mode-selector', component_property='hidden'),
          Output(component_id='scan-pattern-selector', component_property='hidden'),
          Input(component_id='lidar-model-dropdown', component_property='value'),
        #   prevent_initial_call=True
          )
def select_lidar_model(model):
    if model == "AVIA":
        return False, False
    else:
        return True, True

slam_pid = 0
mavros_pid = 0
ptp_pid = 0
slam_children = []

# START SLAM callback
@callback(
    Output(component_id='button-div', component_property='children'),
    Input(component_id='slam-starter', component_property='submit_n_clicks'),
    State('return-mode-dropdown', 'value'),
    State('scan-pattern-dropdown', 'value'),
    State('tty-dropdown', 'value'),
    State('lidar-model-dropdown', 'value'),
    background=True,
    running=[
        (Output("slam-starter-div", "hidden"), True, False),
        (Output("slam-stopper-div", "hidden"), False, True),
    ],
    cancel=[Input("slam-stopper", "submit_n_clicks")],
    manager=background_callback_manager,
    prevent_initial_call=True,
    progress=[Output("ptp-terminal", "value"),
            Output("mavros-terminal", "value"),
            Output("slam-terminal", "value"),
            Output("status-terminal", "value"),
            Output("ptp-pid", "children")]
)
def start_slam(set_progress, # This must be the first argument
            submit_n_clicks: int,
            return_mode: str,
            scan_pattern: str,
            tty: str,
            model: str):
    # User clicked "start SLAM"
    status_msg = "The following configuration was selected:\n -> lidar model: {}\n  -> tty: {}\n  -> return mode: {}\n  -> scan pattern: {}\n".format(model, tty, return_mode, scan_pattern)

    # Retrieve bash scripts
    dir_path = os.path.dirname(os.path.realpath(__file__))
    slam_script_path = os.path.join(dir_path, "scripts/slam-ros2/start_docker_slam_avia.sh")
    mavros_script_path = os.path.join(dir_path, "scripts/slam-ros2/start_mavros.sh")
    ptp_script_path = os.path.join(dir_path, "scripts/slam-ros2/start_ptp.sh")
    start_slam_cmd = "bash " + slam_script_path
    start_mavros_cmd = "bash " + mavros_script_path
    start_ptp_cmd = "bash " + ptp_script_path

    # Start the needed processes
    slam_subprocess = subprocess.Popen(start_slam_cmd, stdout=subprocess.PIPE, shell=True)
    mavros_subprocess = subprocess.Popen(start_mavros_cmd, stdout=subprocess.PIPE, shell=True)
    ptp_subprocess = subprocess.Popen(start_ptp_cmd, stdout=subprocess.PIPE, shell=True)
    os.set_blocking(slam_subprocess.stdout.fileno(), False)
    os.set_blocking(mavros_subprocess.stdout.fileno(), False)
    os.set_blocking(ptp_subprocess.stdout.fileno(), False)
    time.sleep(2) # Wait for the processes to start!!

    global slam_pid, mavros_pid, ptp_pid

    slam_pid = slam_subprocess.pid
    slam_parent = psutil.Process(slam_pid)

    global slam_children
    slam_children = slam_parent.children(recursive=True)
    slam_children_pid = []
    for pid in slam_children:
        slam_children_pid.append(pid.pid)
    mavros_pid = mavros_subprocess.pid
    ptp_pid = ptp_subprocess.pid


    slam_line = ""
    mavros_line = ""
    ptp_line = ""
    log_time = time.time_ns()
    while True:
        elapsed = f'{(time.time_ns() - log_time) / 10**9:.2f}'
        #
        slam_line = slam_subprocess.stdout.readline().decode() + slam_line
        #
        mavros_line = mavros_subprocess.stdout.readline().decode() + mavros_line
        #
        ptp_line = ptp_subprocess.stdout.readline().decode() + ptp_line
        
        set_progress((ptp_line, mavros_line, slam_line, status_msg, ptp_pid))


@callback(
    Output(component_id='dummy-div', component_property='children'),
    Input(component_id='slam-stopper', component_property='submit_n_clicks'),
    State("ptp-pid", "children"),
    prevent_initial_call=True
)
def stop_slam(submit_n_clicks, child_pid):
    subprocess.run("docker container stop fast-lio-slam", shell=True)
    subprocess.run("docker container stop mavros", shell=True)
    # TODO: kill ptp process!! os.kill(ptp_pid, signal.SIGKILL)
    # subprocess.run()

# Run the app
if __name__ == '__main__':
    app.run(debug=True, host="0.0.0.0")
