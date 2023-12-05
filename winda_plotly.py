import numpy as np
import pandas as pd
from dash import Dash, dcc, html, Input, Output, callback, State
import plotly.express as px
from plotly.subplots import make_subplots
# import json
# import copy
# from plotly import __version__
# import plotly.offline as pyo
import plotly.graph_objects as go


stala_mechaniczna = 10
stala_elektryczna = 5
g=9.81 # m/s^2

default_parameters = {
    "L_cewka" : 0.1  ,# H
    "R_cewka" : 0.3  ,# ohm
    "Bezwl_silnik" : 0.3  ,# kgm^2/s^2
    "Tarcie_silnik" : 1  ,#
    "moc_silnika" : 600  ,# W
    "promien_bebna" : 0.3  ,# m
    "masa_windy" : 400  ,# kg
    "masa_obciazenia" : 150  ,# kg
    "masa_przeciwwagi" : 400 ,# kg
    "tarcie_winda_szyb" : 8 ,# ??
    "sprezystosc_liny" : 65000  ,# N/m

    # PID
    "Kp" : 0.7,    # Współczynnik proporcjonalny
    "Ki" : 0.2,    # Współczynnik całkujący
    "Kd" : 0.1,    # Współczynnik różniczkujący
}


def generate_data(time, goal, params = default_parameters):
    czas = time


    df = pd.DataFrame({"Czas":czas,
                       "Prąd":np.zeros_like(czas), 
                       "Napięcie":np.zeros_like(czas),
                       "Prędkość kątowa":np.zeros_like(czas),
                       "Pozycja windy":np.zeros_like(czas),
                       "Prędkość windy":np.zeros_like(czas),
                       "Przyśpieszenie windy":np.zeros_like(czas),
                       "Pozycja przeciwwagi":np.zeros_like(czas),
                       "Prędkość przeciwwagi":np.zeros_like(czas),
                       "Przyśpieszenie przeciwwagi":np.zeros_like(czas),
                       "Docelowe położenie":goal,
                       })


    # Pętla symulacji
    for i in range(len(czas)):
        df["Napięcie"][i] = params["B"]*i**params["A"]
        df["Prąd"][i] = (df["Napięcie"][i]/(2**params["A"]))

    return df
    

czas = np.arange(0,100.01,0.01)
df = pd.DataFrame({"Czas":czas,
                   "Prąd":np.zeros_like(czas), 
                   "Napięcie":np.zeros_like(czas),
                   "Prędkość katowa":np.zeros_like(czas),
                   "Pozycja windy":np.zeros_like(czas),
                   "Prędkość windy":np.zeros_like(czas),
                   "Przyśpieszenie windy":np.zeros_like(czas),
                   "Pozycja przeciwwagi":np.zeros_like(czas),
                   "Prędkość przeciwwagi":np.zeros_like(czas),
                   "Przyśpieszenie przeciwwagi":np.zeros_like(czas),
                   "Docelowe położenie":np.zeros_like(czas),
                   })

external_stylesheets = ['https://codepen.io/chriddyp/pen/bWLwgP.css']
app = Dash(__name__, external_stylesheets=external_stylesheets)


app.layout = html.Div([
    dcc.Slider(0, 5, 1,
               value=1,
               id='slider1'
    ),

    dcc.Slider(0, 1, 0.1,
               value=1,
               id='slider2'
    ),

    html.Button(id='submit-button-state', n_clicks=0, children='Submit'),

    dcc.Graph(id='graph')

])


@callback(Output('graph', 'figure'),
          Input('submit-button-state', 'n_clicks'),
          State('slider1', 'value'),
          State('slider2', 'value'))
def update_figure(n_clicks, input1, input2):
    df = generate_data(params={"A":input1,"B":input2}, time=czas, goal=np.ones_like(czas)*80)   
    fig = make_subplots(rows=3, cols=2)
    fig.update_layout(transition_duration=50)


    fig.add_trace(
        go.Scatter(x=df["Czas"], y=df["Napięcie"]),
        row=1, col=1
    )

    fig.add_trace(
        go.Scatter(x=df["Czas"], y=df["Prąd"]),
        row=1, col=2
    )

    fig.add_trace(
        go.Scatter(x=df["Czas"], y=df["Prędkość kątowa"]),
        row=2, col=1
    )

    fig.add_trace(
        go.Scatter(x=df["Czas"], y=df["Pozycja przeciwwagi"]),
        row=2, col=2
    )

    fig.add_trace(
        go.Scatter(x=df["Czas"], y=df["Pozycja windy"]),
        row=3, col=1
    )
    
    fig.add_trace(
        go.Scatter(x=df["Czas"], y=df["Przyśpieszenie windy"]),
        row=3, col=2
    )


    return fig


if __name__ == '__main__':
    app.run(debug=True)

   
