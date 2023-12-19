# Wersja test import numpy as np
import pandas as pd
from dash import Dash, dcc, html, Input, Output, callback
import plotly.express as px
from plotly.subplots import make_subplots
import plotly.graph_objects as go
import numpy as np
import math

# external_stylesheets = ['https://codepen.io/chriddyp/pen/bWLwgP.css']
app = Dash(__name__)
colors = {
    "Napięcie": "blue",
    "Prąd": "red",
    "Prędkość kątowa": "green",
    "Przyśpieszenie windy": "orange",
    "Przemieszczenie windy": "black",
    "Docelowe położenie": "lightgray",
    "Prędkość windy" : "pink",
}

units = {
    "Czas": "s",
    "Napięcie": "V",
    "Prąd": "A",
    "Prędkość kątowa": "rad/s",
    "Przyśpieszenie windy": "m/s^2",
    "Przemieszczenie windy": "m",
    "Docelowe położenie": "m",
    "Prędkość windy" : "m/s",
}


p = {
    "L_cewka": 0.05,  # H
    "R_cewka": 3,  # ohm
    "bezwladnosc_silnika": 0.3, # kgm^2/s^2
    "stala_mechaniczna": 5,
    "promien_bebna": 0.1,  # m
    "masa_windy": 400,  # kg
    "masa_obciazenia": 100,  # kg
    "tarcie_winda_szyb": 10,
    "wysokosc_pietra": 4,  # m
    "Kp": 100,  # Wzmocnienie regulatora
    "Ti": 50,  # Czas zdwojenia
    "Td": 3,  # Czas wyprzedzenia
    "Tp": 0.01,  # Czas próbkowania
    "pietro_start": 0,  # Wysokość docelowa na którą jedzie winda [m]
    "pietro_koniec": 2,  # Wysokość docelowa na którą jedzie winda [m]
    "czas_symulacji": 30,
}

precyzja = {
    "Czas": -math.ceil(math.log10(abs(p["Tp"]))),
    "Napięcie": 2,
    "Prąd": 2,
    "Prędkość kątowa": 2,
    "Przyśpieszenie windy": 2,
    "Przemieszczenie windy": 2,
    "Docelowe położenie": 2,
    "Prędkość windy" : 2,
}

@callback(
    Output("pietro_koniec", "value"),
    Input("pietro_start", "value"),
    Input("pietro_koniec", "value"),
)
def get_level(start, koniec):
    if start == koniec:
        if koniec == 1:
            return 2
        else:
            return 1
    else:
        return koniec


def generate_data(parameters={}, time=[], goal=[]):
    p.update(parameters)

    G = 9.81

    # Czas symulacji
    czas_symulacji = p["czas_symulacji"]  # s
    Tp = p["Tp"]  # s
    czas = np.arange(0, czas_symulacji + Tp, Tp)

    pozycja_zadana = (
        np.ones_like(czas)
        * (p["pietro_koniec"] - p["pietro_start"])
        * p["wysokosc_pietra"]
    )

    v_max = 2
    
    pozycja_zadana[0] = 0
    for i in range(1,len(pozycja_zadana)):
        if(p["pietro_koniec"]-p["pietro_start"]>0):
            if(pozycja_zadana[i-1]<(p["pietro_koniec"] - p["pietro_start"])* p["wysokosc_pietra"]):
                pozycja_zadana[i]=pozycja_zadana[i-1]+v_max*Tp
        if (p["pietro_koniec"] - p["pietro_start"]<0):
            if (pozycja_zadana[i - 1] > (p["pietro_koniec"] - p["pietro_start"]) * p["wysokosc_pietra"]):
                pozycja_zadana[i] = pozycja_zadana[i - 1] - v_max * Tp

    g = np.ones_like(czas) * G

    prad = np.zeros_like(czas)
    predkosc_katowa = np.zeros_like(czas)
    pozycja = np.zeros_like(czas)
    predkosc_winda = np.zeros_like(czas)
    przyspieszenie_winda = np.zeros_like(czas)

    U = [0]
    suma_uchybow = 0
    poprzedni_uchyb = 0 #pozycja_zadana[0] - pozycja[0]

    # Pętla symulacji
    for i in range(len(czas) - 1):
        # Regulator PID
        uchyb = pozycja_zadana[i] - pozycja[i]
        suma_uchybow += uchyb
        U_pid = p["Kp"] * (
            uchyb
            + Tp*suma_uchybow/p["Ti"]
            + (p["Td"] * (uchyb - poprzedni_uchyb)/ Tp)
        )
        Uz = max(-230, min(230, U_pid))
        poprzedni_uchyb = uchyb
        U.append(Uz)

        # Równania elektryczne silnika
        deltaI_Tp = (1 / p["L_cewka"]) * (
            Uz - p["R_cewka"] * prad[i] - (1/p["stala_mechaniczna"])*predkosc_katowa[i]
        )
        prad[i + 1] = prad[i] + deltaI_Tp * Tp

        # Równania mechaniczne silnika
        M_obc = (p["masa_obciazenia"]) * g[i] * p["promien_bebna"]  # moment obciążenia
        deltaOmega_Tp = (1 / p["bezwladnosc_silnika"]) * (
            +p["stala_mechaniczna"] * prad[i] - predkosc_katowa[i] - M_obc
        )
        predkosc_katowa[i + 1] = predkosc_katowa[i] + deltaOmega_Tp * Tp

        # Równania windy
        pozycja[i + 1] = (
            (
                p["tarcie_winda_szyb"] * predkosc_katowa[i] * p["promien_bebna"]
                + pozycja[i]
                - predkosc_katowa[i] * p["promien_bebna"] * Tp
            )/(p["masa_windy"]+p["masa_obciazenia"])
            * Tp
            * Tp
            + 2 * pozycja[i]
            - pozycja[i - 1]
        )

        # Aktualizacja wartości
        predkosc_winda[i + 1] = (pozycja[i + 1] - pozycja[i]) / Tp
        # predkosc_winda[i + 1] = max(min(predkosc_winda[i+1],v_max),-v_max)
        przyspieszenie_winda[i + 1] = (predkosc_winda[i + 1] - predkosc_winda[i]) / Tp

    return pd.DataFrame(
        {
            "Czas": czas,
            "Prąd": prad,
            "Napięcie": U,
            "Prędkość kątowa": predkosc_katowa,
            "Przemieszczenie windy": pozycja,
            "Prędkość windy": predkosc_winda,
            "Przyśpieszenie windy": przyspieszenie_winda,
            "Docelowe położenie": pozycja_zadana,
        })

    # Za wolne
    # for (key,value) in precyzja.items():
    #     for j,k in df.iterrows():
    #         df.loc[j,key] = round(df.loc[j,key], value)
   


app.layout = html.Div(
    children=[
        html.Div(
            children=[
                html.H1("SYMULATOR WINDY"),
                html.Br(),
                html.H5("Piętro startowe"),
                dcc.Slider(-2, 9, 1, value=0, id="pietro_start"),
                html.H5("Piętro końcowe"),
                dcc.Slider(-2, 9, 1, value=2, id="pietro_koniec"),
                html.H5("Masa obciążenia [kg]"),
                dcc.Slider(0, 200, 50, value=p["masa_obciazenia"], id="masa_obciazenia"),
                html.Br(),
                html.Br(),
                html.H5("Wzmocnienie regulatora"),
                dcc.Slider(0.5, 400, value=p["Kp"], id="Kp"),
                html.H5("Czas zdwojenia"),
                dcc.Slider(1, 100, value=p["Ti"], id="Ti"),
                html.H5("Czas wyprzedzenia"),
                dcc.Slider(0.1, 10, value=p["Td"], id="Td"),
                html.Br(),
                html.H5("Czas próbkowania [s]"),
                dcc.Slider(0.01, 0.03, 0.01, value=p["Tp"], id="Tp"),
                # html.H5("Czas symulacji [s]"),
                # dcc.Slider(10, 60, 10, value=p["czas_symulacji"], id="czas_symulacji"),
                # html.Br(),
                # html.Br(),
                # html.Br(),
            ],
            id="controls",
            className="controls",
            style={"padding": 10, "flex": 1},
        ),
        html.Div(
            [
                dcc.Tabs(
                    id="tab",
                    value="Przemieszczenie windy",
                    children=[
                        dcc.Tab(label="Napięcie", value="Napięcie"),
                        # dcc.Tab(label="Prąd", value="Prąd"),
                        dcc.Tab(label="Prędkość kątowa", value="Prędkość kątowa"),
                        dcc.Tab(label="Prędkość windy", value="Prędkość windy"),
                        dcc.Tab(
                            label="Przyśpieszenie windy", value="Przyśpieszenie windy"
                        ),
                        dcc.Tab(
                            label="Przemieszczenie windy", value="Przemieszczenie windy"
                        ),
                    ],
                ),
                html.Div(
                    id="graph-div",
                    className="graph-div",
                ),
            ],
            style={"padding": 10, "flex": 2},
        ),
    ],
    id="layout",
    className="layout",
    style={"display": "flex", "flexDirection": "row"},
)


@callback(
    Output("graph-div", "children"),
    Input("pietro_start", "value"),
    Input("pietro_koniec", "value"),
    Input("masa_obciazenia", "value"),
    Input("Kp", "value"),
    Input("Ti", "value"),
    Input("Td", "value"),
    Input("Tp", "value"),
    # Input("czas_symulacji", "value"),
    Input("tab", "value"),
)
def update_figure(
    pietro_start, pietro_koniec, masa_obciazenia, Kp, Ti, Td, 
    Tp,
    tab
):
    df = generate_data(
        {
            "pietro_start": pietro_start,
            "pietro_koniec": pietro_koniec,
            "masa_obciazenia" : masa_obciazenia,
            "Kp": Kp,
            "Ti": Ti,
            "Td": Td,
            "Tp" : Tp,
        }
    )

    if tab != "Przemieszczenie windy":
        fig = px.line(
                    df,
                    x="Czas",
                    y=tab,
                    color_discrete_sequence=[colors[tab]],
                    labels={unit:tab+" ["+units[tab]+']' for unit in units},
        )
        fig.update_traces(hovertemplate="Czas: %{x:."+str(precyzja["Czas"])+"f}s<br>"+tab+": %{y:."+str(precyzja[tab])+"f}"+units[tab])
        return dcc.Graph(
            figure=fig,
            )
        
    else:
        fig = make_subplots()
        fig.add_trace(
            go.Scatter(
                x=df["Czas"],
                y=df["Przemieszczenie windy"],
                mode="lines",
                name="Przemieszczenie windy",
                line=dict(color=colors["Przemieszczenie windy"]),
            )
        )
        fig.add_trace(
            go.Scatter(
                x=df["Czas"],
                y=df["Docelowe położenie"],
                mode="lines",
                name="Docelowe przemieszczenie",
                line=dict(color=colors["Docelowe położenie"]),
            )
        )
        fig.update_xaxes(title_text="Czas [s]")
        fig.update_yaxes(title_text="Przemieszczenie [m]")
        fig.update_traces(hovertemplate=
    'Czas: %{x:.'+str(precyzja["Czas"])+'f}s'+
    '<br>Przemieszczenie : %{y:.'+str(precyzja["Przemieszczenie windy"])+'f}m')
        return dcc.Graph(figure=fig)


if __name__ == "__main__":
    app.run(debug=True)
