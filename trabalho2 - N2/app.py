import sys
import time
import numpy as np
import pandas as pd

import serial
from serial.tools import list_ports
from scipy.optimize import minimize
from scipy.interpolate import interp1d
import control as ctl
import matplotlib
matplotlib.use("QtAgg")
import matplotlib.pyplot as plt

from PySide6.QtWidgets import (
    QApplication, QMainWindow, QWidget, QPushButton, QLineEdit,
    QVBoxLayout, QHBoxLayout, QFormLayout, QComboBox, QPlainTextEdit,
    QMessageBox, QFileDialog, QTabWidget, QSplitter, QGroupBox, QLabel, QDialog, QTextBrowser
)
from PySide6.QtCore import Qt
from matplotlib.backends.backend_qt5agg import FigureCanvasQTAgg as FigureCanvas
from matplotlib.backends.backend_qt5agg import NavigationToolbar2QT as NavigationToolbar

###############################################################################
# FUNÇÕES DE LEITURA, IDENTIFICAÇÃO, SINTONIA E GERAÇÃO DE CÓDIGO
###############################################################################
def ler_dados_arduino(porta, baudrate=9600):
    """
    Lê os dados do Arduino continuamente até que o sinal de entrada (u) retorne a zero,
    após ter sido diferente de zero. Retorna arrays NumPy para tempo, u e y.
    """
    ser = serial.Serial(porta, baudrate, timeout=1)
    time.sleep(2)  # Aguarda reset do Arduino
    time_array = []
    u_array = []
    y_array = []
    nonzero_found = False  # Flag que indica se já foi detectado um valor não nulo
    while True:
        line = ser.readline().decode(errors='ignore').strip()
        if not line:
            continue
        parts = line.split(',')
        if len(parts) != 3:
            continue
        try:
            t = float(parts[0])
            u = float(parts[1])
            y = float(parts[2])
        except ValueError:
            continue
        time_array.append(t)
        u_array.append(u)
        y_array.append(y)
        if not nonzero_found and u != 0:
            nonzero_found = True
        # Se já houve um valor não nulo e agora u é zero, interrompe (não inclui a linha com zero)
        if nonzero_found and u == 0:
            break
    ser.close()
    return np.array(time_array), np.array(u_array), np.array(y_array)

def ler_dados_csv(filename):
    """
    Lê todo o arquivo CSV e retorna os dados (tempo, entrada, saida)
    até que, após ter sido não nula, a coluna 'entrada' retorne a zero (não incluindo essa linha).
    Supõe que o CSV possui 3 colunas na ordem: tempo, entrada, saida.
    """
    df = pd.read_csv(filename, sep=",")
    data = df.values  # Converte para array NumPy
    nonzero_found = False
    end_index = len(data)
    for i, row in enumerate(data):
        if not nonzero_found and row[1] != 0:
            nonzero_found = True
        if nonzero_found and row[1] == 0:
            end_index = i
            break
    return data[:end_index, 0], data[:end_index, 1], data[:end_index, 2]

def func_erro(params, time_arr, u_interp, y_arr):
    k, theta = params
    theta = abs(theta)
    t_shifted = np.maximum(time_arr - theta, 0)
    u_delayed = u_interp(t_shifted)
    dt = time_arr[1] - time_arr[0]
    y_model = k * np.cumsum(u_delayed) * dt
    return np.sum((y_model - y_arr) ** 2)

def identificar_k_theta(input_mode, source, baudrate=9600):
    if input_mode == "Arduino":
        time_arr, u_arr, y_arr = ler_dados_arduino(source, baudrate)
    else:
        time_arr, u_arr, y_arr = ler_dados_csv(source)
    if len(time_arr) < 2:
        raise RuntimeError("Dados insuficientes ou falha na leitura.")
    u_interp = interp1d(time_arr, u_arr, kind='linear', fill_value='extrapolate')
    res = minimize(func_erro, [1.0, 1.0], args=(time_arr, u_interp, y_arr), method='Nelder-Mead')
    k_opt, theta_opt = res.x
    return k_opt, abs(theta_opt), time_arr, u_arr, y_arr, u_interp

def imc_pid_ipdt(k, theta, lambd):
    Kp = 2 * (lambd + theta) / (k * (2 * lambd + theta))
    Ti = 2 * (lambd + theta)
    Td = theta / 2.0
    return Kp, Ti, Td

def montar_controlador_pid(Kp, Ti, Td, N=100):
    C_p = ctl.TransferFunction([Kp], [1])
    C_i = ctl.TransferFunction([Kp / Ti], [1, 0])
    C_d = ctl.TransferFunction([Kp * Td * N, 0], [1, N])
    return C_p + C_i + C_d

def discretizar_controlador(C, Ts):
    Cz = ctl.c2d(C, Ts, method='zoh')
    numz = Cz.num[0][0]
    denz = Cz.den[0][0]
    return Cz, numz, denz

def gerar_equacao_diferencas(numz, denz):
    a = denz.copy()
    b = numz.copy()
    a0 = a[0]
    if abs(a0) < 1e-12:
        raise ValueError("a0 ~ 0; controlador inválido.")
    if abs(a0 - 1.0) > 1e-12:
        b = [bi / a0 for bi in b]
        a = [ai / a0 for ai in a]
        a[0] = 1.0
    eq_str = "u[k] = "
    neg_a = [f"({-a[i]:.6f})*u[k-{i}]" for i in range(1, len(a))]
    eq_str += " + ".join(neg_a) if neg_a else "0"
    plus_b = [f"({b[j]:.6f})*e[k-{j}]" for j in range(len(b))]
    eq_str += " + " + " + ".join(plus_b)
    return eq_str

def gerar_codigo_c(a, b):
    termos_uk = []
    for i in range(1, len(a)):
        termos_uk.append(f"({-a[i]:.6f})*uk_{i}")
    termos_ek = []
    for j in range(len(b)):
        if j == 0:
            termos_ek.append(f"({b[j]:.6f})*ek")
        else:
            termos_ek.append(f"({b[j]:.6f})*ek_{j}")
    expr_uk = " + ".join(termos_uk + termos_ek) if (termos_uk + termos_ek) else "0.0"
    linha_uk = f"uk = {expr_uk};"
    codigo_template = f"""
#include <TimerOne.h>

// Declaração de variáveis
volatile float yk;
volatile float ek = 0.0; 
volatile float ek_1 = 0.0;
volatile float ek_2 = 0.0;  // 2º termo de erro atrasado
volatile float uk = 0.0;
volatile float uk_1 = 0.0;
volatile float uk_2 = 0.0;  // 2º termo de controle atrasado

const float ref = 1.0;      // Referência (equivalente a 2.5V analógico, pode ajustar)
const int PWMPIN = 5;       // Pino para gerar PWM
const int AnalogPin = A0;   // Pino para leitura AD 
volatile int pwm;


void setup() {{
  pinMode(PWMPIN, OUTPUT);
  pinMode(AnalogPin, INPUT);
  analogWrite(PWMPIN, 255);
  Timer1.initialize(1000); // 1ms
  Timer1.attachInterrupt(controlador);
  Serial.begin(9600);
}}

void loop() {{
  // Loop vazio: controle via interrupção
}}

void controlador() {{
   yk = analogRead(AnalogPin);
   yk = (5.0 * yk) / 1023.0;
   Serial.print(yk);
   Serial.print("\\t");

   ek = ref - yk;
   Serial.print(ek);
   Serial.print("\\t");

   // Controlador discreto
   {linha_uk}

   Serial.print(uk);
   Serial.print("\\t");

   pwm = (int)(uk * 255.0 / 5.0);
   if (pwm > 255) pwm = 255;
   if (pwm < 0)   pwm = 0;
   analogWrite(PWMPIN, pwm);

   Serial.println(pwm);

   // Atualiza variáveis atrasadas
   uk_2 = uk_1;
   uk_1 = uk;
   ek_2 = ek_1;
   ek_1 = ek;
}}
"""
    return codigo_template

###############################################################################
# Classe para embutir o Matplotlib no Qt
###############################################################################
class MplCanvas(FigureCanvas):
    def __init__(self, parent=None, width=5, height=4, dpi=100):
        self.fig, self.ax = plt.subplots(figsize=(width, height), dpi=dpi)
        super().__init__(self.fig)
        self.setParent(parent)
        self.fig.tight_layout()

###############################################################################
# Tela de Onboarding (Instruções)
###############################################################################
class OnboardingDialog(QDialog):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("Bem-vindo ao Identificador IMC PID")
        self.setMinimumSize(600, 400)
        layout = QVBoxLayout(self)
        
        instructions = """
        <h2>Bem-vindo!</h2>
        <p>Este aplicativo permite identificar os parâmetros de um processo e sintonizar um controlador IMC-PID.</p>
        <p><b>Formato de Dados:</b></p>
        <ul>
          <li><b>Arduino:</b> Envie os dados via porta serial no seguinte formato (CSV):</li>
        </ul>
        <pre>
unsigned long startTime;
float tempoSegundos = 0.0;
int entradaPWM = 0;   // Valor de 0 a 255
float saidaADC = 0.0; // Valor lido no ADC (0..1023 -> 0..5 V)

void setup() {
  Serial.begin(9600);
  pinMode(5, OUTPUT);
  pinMode(A0, INPUT);
  startTime = millis();
}

void loop() {
  entradaPWM = 180; 
  analogWrite(5, entradaPWM);
  int leituraADC = analogRead(A0);
  saidaADC = (5.0 * leituraADC) / 1023.0;
  unsigned long currentTime = millis();
  tempoSegundos = (currentTime - startTime) / 1000.0;
  // Formato: tempo,entradaPWM,saidaADC
  Serial.print(tempoSegundos);
  Serial.print(",");
  Serial.print(entradaPWM);
  Serial.print(",");
  Serial.println(saidaADC);
  delay(50);
}
        </pre>
        <ul>
          <li><b>CSV:</b> Importe um arquivo CSV com 3 colunas: <i>tempo, entrada, saida</i>.</li>
          <li><b>JSON (Opcional):</b> Cada linha deve ser um objeto JSON com chaves <code>"time"</code>, <code>"u"</code> e <code>"y"</code>.</li>
        </ul>
        <p>Após selecionar a fonte de dados e configurar os parâmetros, clique em <b>"Executar"</b> para visualizar:</p>
        <ol>
          <li><b>Modelo Integrador:</b> Gráfico da saída real sobreposta ao modelo identificado: <br><pre>G(s) = k · exp(-θ·s)/s</pre></li>
          <li><b>Malha Fechada:</b> Função de transferência do controlador contínuo, os valores de <i>Kp, Ti e Td</i>, e o gráfico da resposta em malha fechada.</li>
          <li><b>Controlador Discretizado:</b> Função de transferência discreta (via ZOH) e a equação de diferenças.</li>
          <li><b>Código em C:</b> Código para implementação no Arduino.</li>
        </ol>
        <p>Clique em <b>"Continuar"</b> para iniciar.</p>
        """
        text_browser = QTextBrowser()
        text_browser.setHtml(instructions)
        layout.addWidget(text_browser)
        btn_continue = QPushButton("Continuar")
        btn_continue.clicked.connect(self.accept)
        layout.addWidget(btn_continue, alignment=Qt.AlignRight)

###############################################################################
# Janela Principal com 4 Abas e Campos Adicionais (λ e Ts)
###############################################################################
class MainWindow(QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("Trabalho Final - Aplicação de Controle")
        central_widget = QWidget()
        self.setCentralWidget(central_widget)
        main_layout = QVBoxLayout(central_widget)
        splitter = QSplitter(Qt.Horizontal)
        main_layout.addWidget(splitter, stretch=1)
        
        # Painel Esquerdo: Configurações e Botões
        left_widget = QWidget()
        left_layout = QVBoxLayout(left_widget)
        input_group = QGroupBox("Fonte de Dados e Parâmetros")
        input_layout = QFormLayout(input_group)
        
        self.combo_input_method = QComboBox()
        self.combo_input_method.addItems(["Arduino", "CSV"])
        self.combo_input_method.currentTextChanged.connect(self.update_input_fields)
        input_layout.addRow("Método:", self.combo_input_method)
        
        self.combo_portas = QComboBox()
        self.combo_portas.addItems(self.atualizar_lista_portas())
        input_layout.addRow("Porta Serial:", self.combo_portas)
        
        self.edit_baud = QLineEdit("9600")
        input_layout.addRow("Baudrate:", self.edit_baud)
        
        h_csv = QHBoxLayout()
        self.edit_csv = QLineEdit()
        self.edit_csv.setPlaceholderText("Selecione o arquivo CSV...")
        self.edit_csv.setEnabled(False)
        self.btn_browse = QPushButton("Buscar")
        self.btn_browse.setEnabled(False)
        self.btn_browse.clicked.connect(self.on_browse_csv)
        h_csv.addWidget(self.edit_csv)
        h_csv.addWidget(self.btn_browse)
        input_layout.addRow("Arquivo CSV:", h_csv)
        
        self.edit_lambda = QLineEdit("2.0")
        self.edit_lambda.setToolTip(
            "Lambda (λ): Parâmetro de sintonia IMC. Valores maiores tornam o controlador mais conservador, "
            "resultando em uma resposta mais lenta; valores menores tornam o controlador mais agressivo."
        )
        input_layout.addRow("Lambda:", self.edit_lambda)
        
        self.edit_ts = QLineEdit("0.001")
        self.edit_ts.setToolTip(
            "Ts: Período de amostragem em segundos. Deve corresponder ao intervalo entre as medições do sistema "
            "e é utilizado para a discretização do controlador."
        )
        input_layout.addRow("Ts (s):", self.edit_ts)
        
        left_layout.addWidget(input_group)
        
        btn_layout = QHBoxLayout()
        self.btn_atualizar_portas = QPushButton("Atualizar Portas")
        self.btn_atualizar_portas.clicked.connect(self.on_atualizar_portas)
        btn_layout.addWidget(self.btn_atualizar_portas)
        self.btn_executar = QPushButton("Executar")
        self.btn_executar.clicked.connect(self.on_executar)
        btn_layout.addWidget(self.btn_executar)
        left_layout.addLayout(btn_layout)
        
        # Painel Direito: QTabWidget com 4 Abas
        self.tabs = QTabWidget()
        
        # Aba 1: Modelo Integrador
        self.tab_modelo = QWidget()
        modelo_layout = QVBoxLayout(self.tab_modelo)
        # Formatação do modelo com <pre> para monoespaçado:
        self.label_modelo = QLabel("Após a execução, serão exibidos os dados do modelo:\n<pre>G(s) = k · exp(-θ·s)/s</pre>")
        self.label_modelo.setWordWrap(True)
        self.label_modelo.setTextFormat(Qt.RichText)
        modelo_layout.addWidget(self.label_modelo)
        self.canvas_modelo = MplCanvas(self, width=5, height=4, dpi=100)
        toolbar_modelo = NavigationToolbar(self.canvas_modelo, self)
        modelo_layout.addWidget(toolbar_modelo)
        modelo_layout.addWidget(self.canvas_modelo)
        self.tabs.addTab(self.tab_modelo, "Modelo Integrador")
        
        # Aba 2: Malha Fechada
        self.tab_malha = QWidget()
        malha_layout = QVBoxLayout(self.tab_malha)
        # Exibe controlador e parâmetros com formatação HTML
        self.label_malha = QLabel("Controlador Contínuo e Resposta em Malha Fechada")
        self.label_malha.setWordWrap(True)
        self.label_malha.setTextFormat(Qt.RichText)
        malha_layout.addWidget(self.label_malha)
        self.canvas_malha = MplCanvas(self, width=5, height=4, dpi=100)
        toolbar_malha = NavigationToolbar(self.canvas_malha, self)
        malha_layout.addWidget(toolbar_malha)
        malha_layout.addWidget(self.canvas_malha)
        self.tabs.addTab(self.tab_malha, "Malha Fechada")
        
        # Aba 3: Controlador Discretizado
        self.tab_discretizado = QWidget()
        discret_layout = QVBoxLayout(self.tab_discretizado)
        self.text_discretizado = QPlainTextEdit()
        self.text_discretizado.setReadOnly(True)
        discret_layout.addWidget(self.text_discretizado)
        self.tabs.addTab(self.tab_discretizado, "Controlador Discretizado")
        
        # Aba 4: Código em C
        self.tab_codigo = QWidget()
        codigo_layout = QVBoxLayout(self.tab_codigo)
        self.text_codigo = QPlainTextEdit()
        self.text_codigo.setReadOnly(True)
        codigo_layout.addWidget(self.text_codigo)
        btn_save = QPushButton("Salvar Código em Arquivo")
        btn_save.clicked.connect(self.on_salvar_codigo)
        codigo_layout.addWidget(btn_save)
        self.tabs.addTab(self.tab_codigo, "Código em C")
        
        splitter.addWidget(left_widget)
        splitter.addWidget(self.tabs)
        splitter.setStretchFactor(0, 0)
        splitter.setStretchFactor(1, 1)
        
        self.resize(950, 700)
        self.update_input_fields(self.combo_input_method.currentText())
    
    def update_input_fields(self, method):
        if method == "Arduino":
            self.combo_portas.setEnabled(True)
            self.edit_baud.setEnabled(True)
            self.btn_atualizar_portas.setEnabled(True)
            self.edit_csv.setEnabled(False)
            self.btn_browse.setEnabled(False)
        else:
            self.combo_portas.setEnabled(False)
            self.edit_baud.setEnabled(False)
            self.btn_atualizar_portas.setEnabled(False)
            self.edit_csv.setEnabled(True)
            self.btn_browse.setEnabled(True)
    
    def atualizar_lista_portas(self):
        ports = list_ports.comports()
        lista = [p.device for p in ports]
        if not lista:
            lista = ["(Nenhuma Porta Encontrada)"]
        return lista
    
    def on_atualizar_portas(self):
        self.combo_portas.clear()
        self.combo_portas.addItems(self.atualizar_lista_portas())
    
    def on_browse_csv(self):
        filename, _ = QFileDialog.getOpenFileName(self, "Selecione o arquivo CSV", filter="CSV Files (*.csv);;All Files (*.*)")
        if filename:
            self.edit_csv.setText(filename)
    
    def on_executar(self):
        input_mode = self.combo_input_method.currentText()
        
        try:
            lambd = float(self.edit_lambda.text())
        except ValueError:
            self.show_error("Lambda inválido!")
            return
        
        try:
            Ts = float(self.edit_ts.text())
        except ValueError:
            self.show_error("Ts inválido!")
            return
        
        # Limpa conteúdos anteriores
        self.canvas_modelo.ax.clear()
        self.canvas_malha.ax.clear()
        self.text_discretizado.clear()
        self.text_codigo.clear()
        
        try:
            if input_mode == "Arduino":
                porta_serial = self.combo_portas.currentText()
                if "Nenhuma" in porta_serial:
                    self.show_error("Nenhuma porta serial disponível.")
                    return
                try:
                    baud = int(self.edit_baud.text())
                except ValueError:
                    self.show_error("Baudrate inválido!")
                    return
                k_opt, theta_opt, time_data, u_data, y_data, u_interp = identificar_k_theta("Arduino", porta_serial, baud)
            else:
                filename = self.edit_csv.text().strip()
                if not filename:
                    self.show_error("Nenhum arquivo CSV selecionado.")
                    return
                k_opt, theta_opt, time_data, u_data, y_data, u_interp = identificar_k_theta("CSV", filename)
            
            dt = time_data[1] - time_data[0]
            t_shifted = np.maximum(time_data - theta_opt, 0)
            y_model = k_opt * np.cumsum(u_interp(t_shifted)) * dt
            
            ############################
            # Aba 1: Modelo Integrador
            ############################
            modelo_text = f"<b>Processo:</b><br><pre>G(s) = {k_opt:.4f} · exp(-{theta_opt:.4f}·s)/s</pre>"
            self.label_modelo.setText(modelo_text)
            self.canvas_modelo.ax.plot(time_data, y_data, 'b.-', label="Saída Real")
            self.canvas_modelo.ax.plot(time_data, y_model, 'r--', label="Saída Modelada")
            self.canvas_modelo.ax.set_title("Modelo Integrador")
            self.canvas_modelo.ax.set_xlabel("Tempo (s)")
            self.canvas_modelo.ax.set_ylabel("Saída")
            self.canvas_modelo.ax.grid(True)
            self.canvas_modelo.ax.legend()
            self.canvas_modelo.draw()
            
            ############################
            # Aba 2: Malha Fechada
            ############################
            Kp, Ti, Td = imc_pid_ipdt(k_opt, theta_opt, lambd)
            C = montar_controlador_pid(Kp, Ti, Td, N=100)
            malha_text = (
                f"<b>Controlador Contínuo:</b><br><pre>C(s) = {C}</pre><br>"
                f"<b>Parâmetros:</b> Kp = {Kp:.4f}, Ti = {Ti:.4f}, Td = {Td:.4f}"
            )
            self.label_malha.setText(malha_text)
            G_s = ctl.TransferFunction([k_opt], [1, 0])
            closed_loop = ctl.feedback(C * G_s, 1)
            t_cl, y_cl = ctl.step_response(closed_loop)
            self.canvas_malha.ax.plot(t_cl, y_cl, 'g-', label="Resposta ao Degrau")
            self.canvas_malha.ax.set_title("Resposta em Malha Fechada")
            self.canvas_malha.ax.set_xlabel("Tempo (s)")
            self.canvas_malha.ax.set_ylabel("Saída")
            self.canvas_malha.ax.grid(True)
            self.canvas_malha.ax.legend()
            self.canvas_malha.draw()
            
            ############################
            # Aba 3: Controlador Discretizado
            ############################
            Cz, numz, denz = discretizar_controlador(C, Ts)
            eq_str = gerar_equacao_diferencas(numz, denz)
            discret_text = f"Controlador Discreto: C(z) = {Cz}\n\nEquação de Diferenças:\n{eq_str}"
            self.text_discretizado.setPlainText(discret_text)
            
            ############################
            # Aba 4: Código em C
            ############################
            a = denz.copy()
            b = numz.copy()
            if abs(a[0] - 1.0) > 1e-12:
                b = [bi / a[0] for bi in b]
                a = [ai / a[0] for ai in a]
                a[0] = 1.0
            codigo_c = gerar_codigo_c(a, b)
            self.text_codigo.setPlainText(codigo_c)
            
        except Exception as e:
            self.show_error(str(e))
    
    def on_salvar_codigo(self):
        texto = self.text_codigo.toPlainText().strip()
        if not texto:
            self.show_error("Não há código para salvar.")
            return
        filename, _ = QFileDialog.getSaveFileName(self, "Salvar Código", filter="C/INO Files (*.c *.ino);;All Files (*.*)")
        if filename:
            try:
                with open(filename, "w", encoding="utf-8") as f:
                    f.write(texto)
            except Exception as exc:
                self.show_error(f"Erro ao salvar arquivo:\n{exc}")
    
    def show_error(self, msg):
        QMessageBox.critical(self, "Erro", msg)

###############################################################################
# Execução do Aplicativo com Tela de Onboarding
###############################################################################
def main():
    app = QApplication(sys.argv)
    onboarding = OnboardingDialog()
    if onboarding.exec() == QDialog.Accepted:
        window = MainWindow()
        window.show()
        sys.exit(app.exec())
    else:
        sys.exit(0)

if __name__ == "__main__":
    main()
