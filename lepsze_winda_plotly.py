# Wersja test
import numpy as np
import pandas as pd
from dash import Dash, dcc, html, Input, Output, callback, State
# import plotly.express as px
from plotly.subplots import make_subplots
# import json
# import copy
# from plotly import __version__
# import plotly.offline as pyo
import plotly.graph_objects as go

#import numpy as np

p = {
    "L_cewka"           : 0.1  ,      # H
    "R_cewka"           : 0.1  ,      # ohm
    "Bezwl_silnik"      : 0.5  ,      # kgm^2/s^2
    "Tarcie_silnik"     : 1    ,      #
    "stala_mechaniczna" : 10   ,
    "stala_elektryczna" : 5    ,
    "moc_silnika"       : 600  ,      # W
    "promien_bebna"     : 0.5  ,      # m
    "masa_windy"        : 200  ,      # kg
    "masa_obciazenia"   : 150  ,      # kg
    "tarcie_winda_szyb" : 10   ,
    "sprezystosc_liny"  : 1    ,      # N/m
    "Kp"                : 0.7  ,      # Współczynnik proporcjonalny
    "Ki"                : 0.2  ,      # Współczynnik całkujący
    "Kd"                : 0.2  ,      # Współczynnik różniczkujący
    "zadana_wysokosc"   : 80   ,      # Wysokość docelowa na którą jedzie winda [m]
}

def generate_data(parameters={},time=[],goal=[]):
    p.update(parameters)
    

    g=9.81



    # Parametry regulatora PID
    # Kp = 0.    # Współczynnik proporcjonalny
    # Ki = 0.5    # Współczynnik całkujący
    # Kd = 0.3    # Współczynnik różniczkujący

    # Warunki początkowe
    integrala = 0
    poprzedni_blad = 0

    # Czas symulacji
    czas_symulacji = 100  # s
    krok_czasowy = 0.01  # s
    czas = np.arange(0, czas_symulacji + krok_czasowy, krok_czasowy)

    v_max = 5
    # Wartość zadana (położenie docelowe)
    pozycja_zadana = np.ones_like(czas)


    for i in range(len(czas)):
        pozycja_zadana[i] = p["zadana_wysokosc"]

    

    # Inicjalizacja tablic do przechowywania wyników
    prad = np.zeros_like(czas)
    predkosc_katowa = np.zeros_like(czas)
    pozycja = np.zeros_like(czas)
    predkosc_winda = np.zeros_like(czas)
    przyspieszenie_winda = np.zeros_like(czas)

    U = [0]

    # Pętla symulacji
    for i in range(len(czas) - 1):
    # Regulator PID
        blad = pozycja_zadana[i] - pozycja[i]
        integrala += blad * krok_czasowy
        pochodna = (blad - poprzedni_blad) / krok_czasowy
        U_pid = p["Kp"] * blad + p["Ki"] * integrala + p["Kd"] * pochodna
        Uz = max(-230,min(230, U_pid))
        poprzedni_blad = blad


        U.append(Uz)


        # Równania elektryczne silnika
        di_dt = (1 / p["L_cewka"]) * (Uz - p["R_cewka"] * prad[i] - p["Bezwl_silnik"] * predkosc_katowa[i])
        prad[i + 1] = prad[i] + di_dt * krok_czasowy

        # Równania mechaniczne silnika
        M_obc = p["masa_obciazenia"]* g * p["promien_bebna"]# moment obciążenia
        domega_dt = (p["stala_mechaniczna"]/ p["Bezwl_silnik"]) * prad[i] - (p["Tarcie_silnik"]/ p["Bezwl_silnik"]) * predkosc_katowa[i] - (
                1 / p['Bezwl_silnik']) * M_obc
        predkosc_katowa[i + 1] = predkosc_katowa[i] + domega_dt * krok_czasowy

        # Równania windy
        d2x_dt2 = (p["tarcie_winda_szyb"] * predkosc_katowa[i]*p["promien_bebna"] +
                   p["sprezystosc_liny"] * (pozycja[i] - p["promien_bebna"] *
                   np.trapz(predkosc_katowa[:i + 1], dx=krok_czasowy)))/p["masa_windy"]


        # Aktualizacja wartości
        pozycja[i + 1] = pozycja[i] + predkosc_katowa[i] * p["promien_bebna"] * krok_czasowy + 0.5 * d2x_dt2 * krok_czasowy ** 2
        #pozycja[i+1] = max(min(pozycja[i+1], 100),0)
        predkosc_winda[i+1] = max(min((pozycja[i+1]-pozycja[i])/krok_czasowy, v_max),-v_max)
        przyspieszenie_winda[i+1] = (predkosc_winda[i+1]-predkosc_winda[i])/krok_czasowy

    return pd.DataFrame({"Czas":czas,
                           "Prąd": prad,
                           "Napięcie": U,
                           "Prędkość kątowa": predkosc_katowa,
                           "Pozycja windy": pozycja,
                           "Prędkość windy": predkosc_winda,
                           "Przyśpieszenie windy": przyspieszenie_winda,
                           # "Pozycja ladunek":np.zeros_like(czas),
                           # "Prędkość ladunek":np.zeros_like(czas),
                           # "Przyśpieszenie ladunek":np.zeros_like(czas),
                           "Docelowe położenie": pozycja_zadana,
                           })


external_stylesheets = ['https://codepen.io/chriddyp/pen/bWLwgP.css']
app = Dash(__name__, external_stylesheets=external_stylesheets)

app.layout = html.Div([

    html.H1("SYMULATOR WINDY"),
    html.Br(),
    
    html.H5("Masa windy"),
    dcc.Slider(100,1000,50, 
               value=p["masa_windy"],
               id='masa_windy'
    ),
    html.Br(),
    
    html.H5("Masa Obciążenia"),
    dcc.Slider(50,p["masa_windy"],50,
               value=p["masa_obciazenia"],
               id='masa_obciazenia'
    ),
    html.Br(),
    
    html.H5("Tarcie winda"),
    dcc.Slider(0.1,1,0.1,
               value=p["tarcie_winda_szyb"],
               id='tarcie_winda'
    ),
    html.Br(),

    html.H5("Tarcie silnik"),
    dcc.Slider(0.1,1,0.1,
               value=p["Tarcie_silnik"],
               id='tarcie_silnik'
    ),
    html.Br(),

    html.H5("L cewka"),
    dcc.Slider(0.01,0.5,0.05,
               value=p["L_cewka"],
               id='L_cewka'
    ),
    html.Br(),

    html.H5("R cewka"),
    dcc.Slider(0.01,0.5,0.05,
               value=p["R_cewka"],
               id='R_cewka'
    ),
    html.Br(),

    html.H5("bezwl silnik"),
    dcc.Slider(0.1,10,0.5,
               value=p["Bezwl_silnik"],
               id='bezwl_silnik'
    ),
    html.Br(),

    html.H5("promien bebna"),
    dcc.Slider(0.1,1,0.1,
               value=p["promien_bebna"],
               id='promien_bebna'
    ),
    html.Br(),

    
    html.H5("Zadana wysokość"),
    dcc.Slider(10,100,10,
               value=p["zadana_wysokosc"],
               id='zadana_wysokosc'
    ),
    html.Br(),

    html.H5("K calka"),
    dcc.Slider(0.1,1,0.1,
               value=p["Ki"],
               id='Ki'
    ),

    html.H5("K pochodna"),
    dcc.Slider(0.1,1,0.1,
               value=p["Kd"],
               id='Kd'
    ),

    html.H5("K proporcjonalny"),
    dcc.Slider(0.1,1,0.1,
               value=p["Kp"],
               id='Kp'
    ),

    html.Br(),
    html.Br(),
    html.Br(),


    # html.Button(id='submit-button-state', n_clicks=0, children='Submit'),

    dcc.Graph(id='graph')

])


@callback(Output('graph', 'figure'),
          # Input('submit-button-state', 'n_clicks'),
          Input('masa_obciazenia', 'value'),
          Input('masa_windy', 'value'),
          Input('tarcie_winda', 'value'),
          Input('tarcie_silnik', 'value'),
          Input('L_cewka', 'value'),
          Input('R_cewka', 'value'),
          Input('bezwl_silnik', 'value'),
          Input('promien_bebna', 'value'),
          Input('zadana_wysokosc', 'value'),
          Input('Ki', 'value'),
          Input('Kd', 'value'),
          Input('Kp', 'value'),
)
# def update_figure(n_clicks, masa_obciazenia, masa_windy, tarcie_winda, tarcie_silnik, L_cewka, R_cewka, bezwl_silnik, promien_bebna,zadana_wysokosc, Ki, Kd, Kp):
def update_figure(masa_obciazenia, masa_windy, tarcie_winda, tarcie_silnik, L_cewka, R_cewka, bezwl_silnik, promien_bebna,zadana_wysokosc, Ki, Kd, Kp):
    
    df = generate_data({
        "L_cewka" : L_cewka,
        "R_cewka" : R_cewka,
        "Bezwl_silnik" : bezwl_silnik,
        "tarcie_winda_szyb" : tarcie_winda,
        "Tarcie_silnik" : tarcie_silnik,
        "masa_windy" : masa_windy,
        "masa_obciazenia" : masa_obciazenia,
        "zadana_wysokosc" : zadana_wysokosc,
        "Ki" : Ki,
        "Kd" : Kd,
        "Kp" : Kp,
    })   
    fig = make_subplots(rows=3, cols=2,subplot_titles=('Napięcie',  'Prąd', 'Prędkość kątowa', 'Prędkość windy', 'pozycja windy,', 'przyśpieszenie'))
    fig.update_layout(transition_duration=50, autosize=False, width=2000, height=2000)


    fig.add_trace(
        go.Scatter(x=df["Czas"], y=df["Napięcie"], name="Napięcie"),
        row=1, col=1,
    )

    fig.add_trace(
        go.Scatter(x=df["Czas"], y=df["Prąd"], name="Prąd"),
        row=1, col=2
    )

    fig.add_trace(
        go.Scatter(x=df["Czas"], y=df["Prędkość kątowa"], name="prędkość kątowa"),
        row=2, col=1
    )

    fig.add_trace(
        go.Scatter(x=df['Czas'], y=df['Prędkość windy'], name="prędkość windy"),
        row=2, col=2
    )

    fig.add_trace(        
        go.Scatter(x=df["Czas"], y=df["Pozycja windy"], name="Pozycja windy"),
        row=3, col=1
    )

    fig.add_trace(
        go.Scatter(x=df["Czas"], y=df["Docelowe położenie"], name="Cel"),
        row=3, col=1
    )
    
    fig.add_trace(
        go.Scatter(x=df["Czas"], y=df["Przyśpieszenie windy"], name="Przyśpieszenie"),
        row=3, col=2
    )


    return fig


if __name__ == '__main__':
    app.run(debug=True)

 
