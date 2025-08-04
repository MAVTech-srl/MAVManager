# Import packages
# from ast import mod
# from curses import baudrate
from dash import Dash, html, dash_table, dcc, callback, Output, Input, State, DiskcacheManager, set_props
import dash_daq as daq
import plotly.express as px
import subprocess
import numpy as np
import time
import os
import signal
import psutil
import json
import zipfile

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
bash_command = "ls -v /dev/ | awk '/^(tty[A-Za-z])/'" # List all files that begin with tty + letter
subproc = subprocess.run(bash_command, shell=True, check=True, executable='/bin/bash', stdout=subprocess.PIPE)
subproc_stdout = str(subproc.stdout.decode()).split()

baudrate_list = [57600, 115200, 921600]

# File browser
save_file_path = os.path.join(os.environ['HOME'], 'Desktop/rosbag')
files = [f for f in os.listdir(save_file_path) if os.path.isfile(os.path.join(save_file_path, f))]
file_browser_status = ""        # History of file browser operations

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
                html.Div(className="row", style={"padding": "10px"}, children=[
                    html.Br(),
                    dcc.Markdown(''' 
                                Select UART baud rate
                                '''),
                    dcc.Dropdown(baudrate_list, id='baud-dropdown', placeholder='Select baud rate...', clearable=True),
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
                        children=html.Button(className='button hover', id="slam-starter-button", children=['Start SLAM'], style={'color': 'white', 'background': 'green', 'text-align': 'center'}),
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
                html.Div(className="two columns", hidden=True, id='slam-stopping-div', style={"padding": "10px"},  children=[
                        html.Button(className='button', children=['Stopping SLAM...'], style={'color': 'black', 'background': 'orange'}, disabled=True)
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
                    dcc.Textarea(id="status-terminal", readOnly=True, style={'width': '100%', 'height': 300})
                ]),
                html.Br(),
                html.Div(className="row", style={"padding": "10px"}, children=[
                    dcc.Markdown(''' 
                                Debug
                                '''),
                    dcc.Checklist(['External monitor connected'], id="monitor-checklist"),
                    dcc.Checklist(['Save local point cloud'], id="pcd-checklist"),
                    dcc.Checklist(['Save UTM (zone 32) point cloud'], id="utm-pcd-checklist", value=['Save UTM (zone 32) point cloud']),
                    dcc.Checklist(['Convert livox custom cloud to ROS cloud'], id="convert-checklist"),

                    html.Br(),
                    html.Button(className='button', children=['Kill old slam session'], id='kill-old-session')
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
        ]),
        dcc.Tab(label='File browser', children=[
            html.Div(className="six columns", children=[
                html.Br(),
                html.Div(className="row", style={"padding": "10px"}, children=[
                    html.Div(children=[html.H2("File browser")], style={"padding": "10px"}),
                    # dcc.Dropdown(
                    #     id="dropdown",
                    #     options=[{"label": x, "value": x} for x in files],
                    #     value=files[0],
                    #     )
                    html.Div(className="row", id="file-buttons", style={"padding": "10px"},  children=[
                        html.Div(className="two columns", id='file-refresh-div', style={"padding": "10px"},  children=[
                            html.Button(className='button', id='refresh-button', children=['Refresh'], style={'color': 'white', 'background': 'green'}),
                        ]),
                        html.Div(className="four columns", id='file-download-div', style={"padding": "10px"},  children=[
                            html.Button(className='button', id='download-button', children=['Download selected files'], style={'color': 'white', 'background': 'blue'}),
                            dcc.Download(id='download'),
                        ]),
                        html.Div(className="four columns", id='file-delete-div', style={"padding": "10px"},  children=[
                            dcc.ConfirmDialogProvider(
                                html.Button(className='button', id='delete-button', children=['Delete selected files'], style={'color': 'white', 'background': 'orange'}),
                                id='delete-button-dialog',
                                message='Do you want to delete these files?'
                            ),
                        ]),
                    ]),
                    html.Br(),
                    html.Div(className="row", id="file-selector", style={"padding": "10px"},  children=[
                        # dcc.Checklist([{"label": x, "value": x} for x in files], id='file-checklist')
                        dcc.Checklist([{"label": html.Div([
                            html.Span(x, style={'width': '500px', 'display': 'inline-block', 'flexShrink': '0', 'whiteSpace': 'nowrap'}, id='file-name'),
                            html.Span("{:.2f}MB".format(os.stat(os.path.join(save_file_path, x)).st_size / 10**6), id='file-size')
                        ], style={'display': 'inline-flex', 'alignItems': 'center'}),
                        "value": x} for x in files], id='file-checklist', inputStyle={"marginRight": "8px"})
                    ]),
                ]),
            ]),
            html.Div(className="six columns", style={"padding": "10px"}, children=[
                html.Div(className="row", style={"padding": "10px"}, children=[
                    dcc.Markdown("File browser output"),
                    dcc.Textarea(id="file-browser-terminal", readOnly=True, style={'width': '100%', 
                                                                            'height': 200}),
                ]),
            ])
        ])
    ])

]

def update_lidar_config(params: dict):
    # Lidar AVIA
    mode = 0
    pattern = 1
    if params["model"] == 'AVIA':
        dir_path = os.path.dirname(os.path.realpath(__file__))
        path = os.path.join(dir_path, "scripts/slam-ros2/fast_lio_slam/config/livox_lidar_config.json") 
        # print(path)
        if params["return_mode"] == 'First single':
            mode = 0
        elif params["return_mode"] == 'Strongest single':
            mode = 1
        elif params["return_mode"] == 'Dual':
            mode = 2
        
        if params["scan_pattern"] == 'Repetitive':
            pattern = 1
        else:
            pattern = 0

        with open(path, 'r') as f:
            data = json.load(f)
            
            data['lidar_config'][0]['return_mode'] = mode
            # print(mode)
            data['lidar_config'][0]['scan_pattern'] = pattern
            # print(pattern)

        with open(path, 'w') as f:
            json.dump(data, f, indent=2)


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
    
# DOWNLOAD FILES callback
@callback(
        Output("download", "data"),
        Input("download-button", "n_clicks"),
        State("file-checklist", 'value'),
        prevent_initial_call=True
)
def download_files(n_clicks, file_list):
    file_paths = []
    for name in file_list:
        file_paths.append(os.path.join(save_file_path, name))
    zf = zipfile.ZipFile("/tmp/clouds.zip", mode="w")
    try:
        for name in file_list:
            zf.write(os.path.join(save_file_path, name), name)

    except FileNotFoundError:
        print("An error occurred")
    finally:
        zf.close()
    return dcc.send_file("/tmp/clouds.zip")

# CANCEL FILES callback
@callback(
        Output('file-selector', 'children', allow_duplicate=True),
        Output('file-browser-terminal', 'value'),
        Input('delete-button-dialog', 'submit_n_clicks'),
        State("file-checklist", 'value'),
        prevent_initial_call=True,
)
def delete_files(n_clicks, file_list: list):
    # Check the size of selected files to delete and then delete it
    freed_space = 0
    for file in file_list:
        freed_space = freed_space + os.stat(os.path.join(save_file_path, file)).st_size
        os.remove(os.path.join(save_file_path, file))
    # Update file list
    updated_files = [f for f in os.listdir(save_file_path) if os.path.isfile(os.path.join(save_file_path, f))]
    # Update the terminal output
    global file_browser_status
    file_browser_status = file_browser_status + "Deleted {} files ({:.2f}MB freed).".format(len(file_list), freed_space / 10**6) + "\n"
    # return updated_files, file_browser_status
    return dcc.Checklist([{"label": html.Div([
                            html.Span(x, style={'width': '500px', 'display': 'inline-block', 'flexShrink': '0', 'whiteSpace': 'nowrap'}, id='file-name'),
                            html.Span("{:.2f}MB".format(os.stat(os.path.join(save_file_path, x)).st_size / 10**6), id='file-size')
                        ], style={'display': 'inline-flex', 'alignItems': 'center'}),
                        "value": x} for x in updated_files], id='file-checklist', inputStyle={"marginRight": "8px"}), file_browser_status

# REFRESH FILES callback
@callback(
        # Output('file-checklist', 'options', allow_duplicate=True),
        Output('file-selector', 'children'),
        Input('refresh-button', 'n_clicks'),
        prevent_initial_call=True
)
def refresh_files(n_clicks):
    # Refresh file list
    updated_files = [f for f in os.listdir(save_file_path) if os.path.isfile(os.path.join(save_file_path, f))]
    # return updated_files
    return dcc.Checklist([{"label": html.Div([
                                html.Span(x, style={'width': '500px', 'display': 'inline-block', 'flexShrink': '0', 'whiteSpace': 'nowrap'}),
                                html.Span("{:.2f}MB".format(os.stat(os.path.join(save_file_path, x)).st_size / 10**6))
                            ], style={'display': 'inline-flex', 'alignItems': 'center'}),
                            "value": x} for x in updated_files], id='file-checklist', inputStyle={"marginRight": "8px"})


# START SLAM callback
@callback(
    Output(component_id='button-div', component_property='children'),
    Input(component_id='slam-starter', component_property='submit_n_clicks'),
    State('return-mode-dropdown', 'value'),
    State('scan-pattern-dropdown', 'value'),
    State('tty-dropdown', 'value'),
    State('baud-dropdown', 'value'),
    State('lidar-model-dropdown', 'value'),
    State('monitor-checklist', 'value'),
    State('pcd-checklist', 'value'),
    State('utm-pcd-checklist', 'value'),
    State('convert-checklist', 'value'),
    background=True,
    running=[
        (Output("slam-starter-div", "hidden"), True, True),
        (Output("slam-stopper-div", "hidden"), False, True),
        (Output(component_id="slam-stopping-div", component_property="hidden"), True, False)
    ],
    cancel=[Input("slam-stopper", "submit_n_clicks")],
    manager=background_callback_manager,
    prevent_initial_call=True,
    progress=[Output("mavros-terminal", "value"),
            Output("slam-terminal", "value"),
            Output("status-terminal", "value")]
)
def start_slam(set_progress, # This must be the first argument
            submit_n_clicks: int,
            return_mode: str,
            scan_pattern: str,
            tty: str,
            baud: int,
            model: str,
            ext_monitor: list,
            save_local: list,
            save_utm: list,
            convert_pc: list):
    # User clicked "start SLAM"
    init_msg = '==== SLAM is starting... ====\n\n'
    set_progress(('', '', init_msg))
    # Customize the lidar config files as per user choices
    lidar_params = {"model": model,
                    "scan_pattern": scan_pattern,
                    "return_mode": return_mode
                    }
    update_lidar_config(lidar_params)

    # Retrieve bash scripts
    dir_path = os.path.dirname(os.path.realpath(__file__))
    
    if model == "AVIA":
        device = "avia"
    elif model == "MID360":
        device = "mid360"

    slam_script_path = os.path.join(dir_path, "scripts/slam-ros2/fast_lio_slam/start_docker_slam_"+ device +".sh")
    mavros_script_path = os.path.join(dir_path, "scripts/slam-ros2/fast_lio_slam/start_mavros.sh")
    start_slam_cmd = "bash " + slam_script_path
    start_mavros_cmd = "bash " + mavros_script_path

    # Add arguments to commands
    if ext_monitor and ext_monitor[0] == 'External monitor connected':
        start_slam_cmd = start_slam_cmd + " --external-monitor=yes"

    if save_local and save_local[0] == 'Save local point cloud':
        start_slam_cmd = start_slam_cmd + " --save-pcd-cloud=yes"  

    if save_utm and save_utm[0] == 'Save UTM (zone 32) point cloud':
        start_slam_cmd = start_slam_cmd + " --save-utm-pcd-cloud=yes"
    else:       # Default is yes, so I need to switch it to off if user un-ticks the checkbox
        start_slam_cmd = start_slam_cmd + " --save-utm-pcd-cloud=no"
    
    if convert_pc and convert_pc[0] == 'Convert livox custom cloud to ROS cloud':
        start_slam_cmd = start_slam_cmd + " --convert-livox-cloud=yes"

    # Start the needed processes
    slam_subprocess = subprocess.Popen(start_slam_cmd, stdout=subprocess.PIPE, shell=True)
    mavros_subprocess = subprocess.Popen(start_mavros_cmd + " " + tty + " " + str(baud) , stdout=subprocess.PIPE, shell=True)
    os.set_blocking(slam_subprocess.stdout.fileno(), False)
    os.set_blocking(mavros_subprocess.stdout.fileno(), False)
    time.sleep(2) # Wait for the processes to start!!

    slam_line = ""
    mavros_line = ""
    
    log_time = time.time()
    while True:
        elapsed = f'{(time.time() - log_time):.0f}'
        status_msg = init_msg + "The following configuration was selected:\n  -> lidar model: {}\n  -> tty: {}\n  -> return mode: {}\n  -> scan pattern: {}\n" \
            "\n==== SLAM is running ====\n\n==== Elapsed time: {} s ====\n".format(model, 
                                                                                   tty, 
                                                                                   return_mode if model=='avia' else None, 
                                                                                   scan_pattern if model=='avia' else None, 
                                                                                   elapsed)
        #
        slam_line = slam_subprocess.stdout.readline().decode() + slam_line
        #
        mavros_line = mavros_subprocess.stdout.readline().decode() + mavros_line
        
        set_progress((mavros_line, slam_line, status_msg))
        # time.sleep(0.2)


@callback(
    # Output(component_id='dummy-div', component_property='children'),
    Output("status-terminal", "value"),
    Output("slam-starter-div", "hidden"),
    Output("slam-stopper-div", "hidden"),
    Output(component_id="slam-stopping-div", component_property="hidden"),
    Input(component_id='slam-stopper', component_property='submit_n_clicks'),
    Input('kill-old-session', 'n_clicks'),
    background=True,
    manager=background_callback_manager,
    prevent_initial_call=True,
    progress=[Output("status-terminal", "value"),
              Output("slam-starter-div", "hidden"),
              Output("slam-stopper-div", "hidden"),
              Output(component_id="slam-stopping-div", component_property="hidden")]
)
def stop_slam(set_progress, submit_n_clicks, n_clicks):
    set_progress(("==== Shutting down SLAM process, please wait... ====", True, True, False))
    subprocess.run("docker kill --signal SIGINT fast-lio-slam", shell=True, stdout=subprocess.DEVNULL)
    subprocess.run("docker container stop mavros", shell=True, stdout=subprocess.DEVNULL)
    return "==== SLAM process gracefully shut down! ====", False, True, True

# Run the app
if __name__ == '__main__':
    app.run(debug=True, host="0.0.0.0")
