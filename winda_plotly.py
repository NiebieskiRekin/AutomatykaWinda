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

czas_symulacji = 100  # s
krok_czasowy = 0.01  # s
stala_mechaniczna = 10
stala_elektryczna = 5
g=9.81 # m/s^2

parameters = {
    # Układ
    "L_cewka"           : 0.1  ,  # H
    "R_cewka"           : 0.3  ,  # ohm
    "Bezwl_silnik"      : 0.3  ,  # kgm^2/s^2
    "Tarcie_silnik"     : 1  ,    #
    "moc_silnika"       : 600  ,  # W
    "opor_silnik"       : 5,      # ohm
    "promien_bebna"     : 0.3  ,  # m
    "masa_windy"        : 400  ,  # kg
    "masa_obciazenia"   : 150  ,  # kg
    "masa_przeciwwagi"  : 400 ,   # kg
    "tarcie_winda_szyb" : 8 ,     # ??
    "sprezystosc_liny"  : 65000  ,# N/m

    # PID
    "Kp"                : 0.7,    # Współczynnik proporcjonalny
    "Ki"                : 0.2,    # Współczynnik całkujący
    "Kd"                : 0.1,    # Współczynnik różniczkujący

    # Czas symulacji
    "czas_symulacji"    : 100,    # s
    "krok_czasowy"      : 0.01,   # s

}


def generate_data(time, goal, params = parameters):
    czas = time

    integrala = 0
    poprzedni_blad = 0


    U = np.zeros_like(czas)
    I = np.zeros_like(czas)
    Omega = np.zeros_like(czas)
    a_winda = np.zeros_like(czas)
    F_winda = np.zeros_like(czas)
    v_winda = np.zeros_like(czas)
    x_winda = np.zeros_like(czas)
    # a_przeciwwaga = np.zeros_like(czas)
    # F_przeciwwaga = np.zeros_like(czas)
    # v_przeciwwaga = np.zeros_like(czas)
    # x_przeciwwaga = np.zeros_like(czas)
    a_ladunek = np.zeros_like(czas)
    F_ladunek = np.zeros_like(czas)
    v_ladunek = np.zeros_like(czas)
    x_ladunek = np.zeros_like(czas)

    # Pętla symulacji
    for i in range(len(czas)-1):

        blad = goal[i] - x_winda[i]
        integrala += blad * krok_czasowy
        pochodna = (blad - poprzedni_blad) / krok_czasowy
        U_pid = parameters["Kp"] * blad + parameters["Ki"] * integrala + parameters["Kd"] * pochodna
        Uz = max(30,min(30, U_pid))
        poprzedni_blad = blad
        U[i] = Uz


        V = 500
        L = 2.90e-3
        Re = 0.445
        R = 5.65
        # L = parameters["L_cewka"]
        # Re = parameters["R_cewka"]
        # R = parameters["opor_silnik"]
        Ka = 2.39e-2 # Nm/A stala elektromechaniczna przekladni
        B1 = 980 # Tarcie
        B2 = 300
        # J = parameters["Bezwl_silnik"]
        J = 0.481
        Beta = 300
        B = Beta * V
        K1 = 145000 # wsp sprężystości
        K2 = 2500 # wsp sprężystości
        # r = parameters["promien_bebna"]
        r = 0.1
        Torque = 158e-9
        n1 = 1430
        n2 = 190
        # mc = parameters["masa_windy"]
        m_winda = 80
        m_ladunek = 500
        
    
        dI_dt: float = U[i]/L - (Re+R)*I[i]/L  - (n1*Omega[i])/(Ka*L*n2)
        I[i+1] = I[i] + dI_dt * krok_czasowy

        dOmega_dt: float = Torque/J + (n2*I[i])/(n1*J*Ka) - B1*Omega[i]/J
        Omega[i+1] = Omega[i] + dOmega_dt*krok_czasowy
        
        dF_winda_dt: float = (K1*Omega[i]/r - K1 * v_winda[i])
        F_winda[i+1] = F_winda[i] + dF_winda_dt*krok_czasowy
        
        dF_ladunek_dt = K2*v_winda[i] - K2*v_ladunek[i]
        F_ladunek[i+1] = F_ladunek[i] + dF_ladunek_dt*krok_czasowy

        dV_winda_dt: float = F_winda[i]/m_winda - (B*v_winda[i-1])/m_winda - F_ladunek[i]/m_winda - (B2*v_winda[i])/m_winda + (B2*v_ladunek[i])/m_winda

        a_winda[i+1] = dV_winda_dt
        v_winda[i+1] = a_winda[i+1]*krok_czasowy + v_winda[i]
        x_winda[i+1] = v_winda[i+1]*krok_czasowy + x_winda[i]       

        dV_ladunek_dt: float = F_ladunek[i]/m_ladunek + (B2*v_winda[i])/m_ladunek - (B2*v_ladunek[i])/m_ladunek 

        a_ladunek[i+1] = dV_ladunek_dt
        v_ladunek[i+1] = a_ladunek[i+1]*krok_czasowy + v_ladunek[i]
        x_ladunek[i+1] = v_ladunek[i+1]*krok_czasowy + x_ladunek[i]       



    df = pd.DataFrame({"Czas":czas,
                       "Prąd":I, 
                       "Napięcie":U,
                       "Prędkość kątowa":Omega,
                       "Pozycja windy":x_winda,
                       "Prędkość windy":v_winda,
                       "Przyśpieszenie windy":a_winda,
                       "Pozycja ladunek":x_ladunek,
                       "Prędkość ladunek":v_ladunek,
                       "Przyśpieszenie ladunek":a_ladunek,
                       "Docelowe położenie":goal,
    })

    with open("dane.csv","w") as f:
      f.write(df.to_csv(index=False))



    return df
    

czas = np.arange(0,czas_symulacji+krok_czasowy,krok_czasowy)
df = pd.DataFrame({"Czas":czas,
                   "Prąd":np.zeros_like(czas), 
                   "Napięcie":np.zeros_like(czas),
                   "Prędkość katowa":np.zeros_like(czas),
                   "Pozycja windy":np.zeros_like(czas),
                   "Prędkość windy":np.zeros_like(czas),
                   "Przyśpieszenie windy":np.zeros_like(czas),
                   "Pozycja ladunek":np.zeros_like(czas),
                   "Prędkość ladunek":np.zeros_like(czas),
                   "Przyśpieszenie ladunek":np.zeros_like(czas),
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
        go.Scatter(x=df["Czas"], y=df["Pozycja ladunek"]),
        row=2, col=2
    )

    fig.add_trace(
        go.Scatter(x=df["Czas"], y=df["Pozycja windy"]),
        row=3, col=1
    )

    fig.add_trace(
        go.Scatter(x=df["Czas"], y=df["Docelowe położenie"]),
        row=3, col=1
    )
    
    fig.add_trace(
        go.Scatter(x=df["Czas"], y=df["Przyśpieszenie windy"]),
        row=3, col=2
    )


    return fig


if __name__ == '__main__':
    app.run(debug=True)

   
