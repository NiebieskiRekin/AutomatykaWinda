# Wersja test import numpy as np
import pandas as pd
from dash import Dash, dcc, html, Input, Output, callback, State
import plotly.express as px
from plotly.subplots import make_subplots
import plotly.graph_objects as go
import numpy as np

# external_stylesheets = ['https://codepen.io/chriddyp/pen/bWLwgP.css']
app = Dash(__name__)
colors = {
    "Napięcie": "blue",
    "Prąd": "red",
    "Prędkość kątowa": "green",
    "Przyśpieszenie windy": "orange",
    "Przemieszczenie windy": "black",
    "Docelowe położenie": "lightgray",
}

units_labels = {
    "Czas" : "Czas [s]",
    "Napięcie": "Napięcie [V]",
    "Prąd": "Prąd [A]",
    "Prędkość kątowa": "Prędkość kątowa [rad/s]",
    "Przyśpieszenie windy": "Przyśpieszenie windy [m/s^2]",
    "Przemieszczenie windy": "Przemieszczenie windy [m]",
    "Docelowe położenie": "Docelowe przemieszczenie [m]",
}

p = {
    "L_cewka": 0.1,  # H
    "R_cewka": 0.3,  # ohm
    "Bezwl_silnik": 0.3,  # kgm^2/s^2
    "Tarcie_silnik": 1,  #
    "stala_mechaniczna": 10,
    "stala_elektryczna": 5,
    "moc_silnika": 600,  # W
    "promien_bebna": 0.1,  # m
    "masa_windy": 400,  # kg
    "masa_obciazenia": 150,  # kg
    "tarcie_winda_szyb": 8,
    "sprezystosc_liny": 1,  # N/m
    "Kp": 0.7,  # Współczynnik proporcjonalny
    "Ki": 0.6,  # Współczynnik całkujący
    "Kd": 0.02,  # Współczynnik różniczkujący
    "pietro_start": 0,  # Wysokość docelowa na którą jedzie winda [m]
    "pietro_koniec": 2,  # Wysokość docelowa na którą jedzie winda [m]
    "Czas symulacji": 30,
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

    wysokosc_pietra = 4  # m

    G = 9.81

    # Warunki początkowe
    integrala = 0
    pochodna = 0

    # Czas symulacji
    czas_symulacji = p["Czas symulacji"]  # s
    krok_czasowy = 0.01  # s
    czas = np.arange(0, czas_symulacji + krok_czasowy, krok_czasowy)

    v_max = 100

    pozycja_zadana = (
        np.ones_like(czas) * (p["pietro_koniec"] - p["pietro_start"]) * wysokosc_pietra
    )
    # for i in range(1,len(czas)):
    #     if (pozycja_zadana[i-1] < p["zadana_wysokosc"] and 0):
    #         pozycja_zadana[i] = pozycja_zadana[i-1] + (v_max/100)*krok_czasowy
    #     else :
    #         pozycja_zadana[i] = p["zadana_wysokosc"]

    g = np.ones_like(czas) * G

    prad = np.zeros_like(czas)
    predkosc_katowa = np.zeros_like(czas)
    pozycja = np.zeros_like(czas)
    predkosc_winda = np.zeros_like(czas)
    przyspieszenie_winda = np.zeros_like(czas)

    U = [0]
    poprzedni_blad = pozycja_zadana[0] - pozycja[0]

    # Pętla symulacji
    for i in range(len(czas) - 1):
        # Regulator PID
        blad = pozycja_zadana[i] - pozycja[i]
        integrala += blad * krok_czasowy
        pochodna = (blad - poprzedni_blad) / krok_czasowy
        U_pid = p["Kp"] * blad + p["Ki"] * integrala + p["Kd"] * pochodna
        Uz = max(-230, min(230, U_pid))
        # poprzedni_blad = blad

        U.append(Uz)

        # Równania elektryczne silnika
        di_dt = (1 / p["L_cewka"]) * (
            Uz - p["R_cewka"] * prad[i] - p["Bezwl_silnik"] * predkosc_katowa[i]
        )
        prad[i + 1] = prad[i] + di_dt * krok_czasowy
        # prad[i+1] = min(max(prad[i+1],-70),70)

        # Równania mechaniczne silnika
        M_obc = p["masa_obciazenia"] * g[i] * p["promien_bebna"]  # moment obciążenia
        domega_dt = (
            (p["stala_mechaniczna"] / p["Bezwl_silnik"]) * prad[i]
            - (p["Tarcie_silnik"] / p["Bezwl_silnik"]) * predkosc_katowa[i]
            - (1 / p["Bezwl_silnik"]) * M_obc
        )
        predkosc_katowa[i + 1] = predkosc_katowa[i] + domega_dt * krok_czasowy

        # Równania windy
        d2x_dt2 = (
            p["tarcie_winda_szyb"] * predkosc_katowa[i] * p["promien_bebna"]
            + p["sprezystosc_liny"]
            * (
                pozycja[i]
                - p["promien_bebna"]
                * np.trapz(predkosc_katowa[: i + 1], dx=krok_czasowy)
            )
        ) / p["masa_windy"]

        # Aktualizacja wartości
        pozycja[i + 1] = (
            pozycja[i]
            + predkosc_katowa[i] * p["promien_bebna"] * krok_czasowy
            + 0.5 * d2x_dt2 * krok_czasowy**2
        )
        # pozycja[i+1] = max(min(pozycja[i+1], 100),0)
        predkosc_winda[i + 1] = max(
            min((pozycja[i + 1] - pozycja[i]) / krok_czasowy, v_max), -v_max
        )
        przyspieszenie_winda[i + 1] = (
            predkosc_winda[i + 1] - predkosc_winda[i]
        ) / krok_czasowy

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
        }
    )


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
                html.Br(),
                html.H5("K proporcjonalny"),
                dcc.Slider(0.1, 1, 0.1, value=p["Kp"], id="Kp"),
                html.H5("K calka"),
                dcc.Slider(0.1, 1, 0.1, value=p["Ki"], id="Ki"),
                html.H5("K pochodna"),
                dcc.Slider(0.01, 0.1, 0.01, value=p["Kd"], id="Kd"),
                html.Br(),
                html.H5("Czas symulacji [s]"),
                dcc.Slider(10, 60, 5, value=p["Czas symulacji"], id="Czas symulacji"),
                html.Br(),
                html.Br(),
                html.Br(),
            ],
            id="controls",
            className="controls",
            style={"padding": 10, "flex": 1},
        ),
        html.Div(
            [
                dcc.Tabs(
                    id="tab",
                    value="Napięcie",
                    children=[
                        dcc.Tab(label="Napięcie", value="Napięcie"),
                        dcc.Tab(label="Prąd", value="Prąd"),
                        dcc.Tab(label="Prędkość kątowa", value="Prędkość kątowa"),
                        dcc.Tab(
                            label="Przyśpieszenie windy", value="Przyśpieszenie windy"
                        ),
                        dcc.Tab(label="Przemieszczenie windy", value="Przemieszczenie windy"),
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
    Input("Kp", "value"),
    Input("Ki", "value"),
    Input("Kd", "value"),
    Input("Czas symulacji", "value"),
    Input("tab", "value"),
)
def update_figure(pietro_start, pietro_koniec, Kp, Ki, Kd, czas_symulacji, tab):
    df = generate_data(
        {
            "pietro_start": pietro_start,
            "pietro_koniec": pietro_koniec,
            "Kp": Kp,
            "Ki": Ki,
            "Kd": Kd,
            "Czas symulacji": czas_symulacji,
        }
    )

    if tab != "Przemieszczenie windy":
        return dcc.Graph(
            figure=(
                px.line(
                    df,
                    x="Czas",
                    y=tab,
                    color_discrete_sequence=[colors[tab]],
                    labels = units_labels,
                )
            )
        )
    else:
        fig = go.Figure(
                px.line(
                    df,
                    x="Czas",
                    y="Przemieszczenie windy",
                    color_discrete_sequence=[colors["Przemieszczenie windy"]],
                    labels = units_labels,
                ).data
                + px.line(
                    df,
                    x="Czas",
                    y="Docelowe położenie",
                    color_discrete_sequence=[colors["Docelowe położenie"]],
                    labels = units_labels,
                ).data)
        fig.update_xaxes(title_text="Czas [s]")
        fig.update_yaxes(title_text="Przemieszczenie [m]")
        return dcc.Graph(
            figure = fig
        )


if __name__ == "__main__":
    app.run(debug=True)
