import numpy as np
import matplotlib.pyplot as plt
import control as ctrl
from scipy.signal import savgol_filter
from scipy.optimize import curve_fit

def calcular_parametros_ziegler_nichols_auto(response, time):
    """
    Estima os parâmetros Td, T1 e K usando ajuste automático baseado na inclinação máxima,
    com definição automática de polyorder e window_length.

    Parâmetros:
    - response: Resposta ao degrau do sistema.
    - time: Vetor de tempo correspondente à resposta.

    Retorna:
    - Td: Tempo de inflexão (interseção da tangente com o eixo x).
    - T1: Constante de tempo (interseção da tangente com o valor final).
    - K: Amplitude máxima da resposta ao degrau.
    """
    if len(response) != len(time):
        raise ValueError("Os vetores 'response' e 'time' devem ter o mesmo tamanho.")

    # Definir automaticamente o window_length como 5% do tamanho dos dados (mínimo 5, deve ser ímpar)
    window_length = max(5, len(response) // 20)
    if window_length % 2 == 0:
        window_length += 1  # Garantir que seja ímpar

    # Definir automaticamente polyorder como o menor valor entre 3 e window_length - 1
    polyorder = min(3, window_length - 1)

    # Calcular a derivada suavizada
    diff_response = np.gradient(response, time)
    diff_response_smooth = savgol_filter(diff_response, window_length, polyorder)

    # Identificar o ponto de inflexão
    max_slope_index = np.argmax(diff_response_smooth)
    slope_at_inflexion = diff_response_smooth[max_slope_index]
    time_inflexion = time[max_slope_index]
    y_inflexion = response[max_slope_index]

    # Ajuste linear ao redor do ponto de inflexão
    def linear_fit(t, m, b):
        return m * t + b

    fit_span = max(1, len(response) // 20)  # Ajuste adaptativo (5% dos dados ou mais)
    t_fit = time[max(0, max_slope_index - fit_span):min(len(time), max_slope_index + fit_span)]
    y_fit = response[max(0, max_slope_index - fit_span):min(len(response), max_slope_index + fit_span)]

    params, _ = curve_fit(linear_fit, t_fit, y_fit)
    m, b = params

    Td = -b / m
    K = response[-1]
    T1 = (K - y_inflexion) / m

    if Td <= 0 or T1 <= 0 or K <= 0:
        raise ValueError("Parâmetros inválidos (Td, T1, K devem ser positivos).")

    return Td, T1, K, slope_at_inflexion, y_inflexion


def calcular_parametros_controlador(tipo, Td, T1, K):
    """
    Calcula os parâmetros do controlador (Kp, Ki, Kd) com base no tipo e nos parâmetros de Ziegler-Nichols.

    Parameters:
    - tipo: Tipo de controlador ('P', 'PI', 'PID').
    - Td: Tempo de inflexão.
    - T1: Constante de tempo.
    - K: Amplitude máxima da resposta ao degrau.

    Returns:
    - Kp, Ki, Kd: Parâmetros do controlador.
    """
    if tipo.upper() == 'P':
        Kp = K * Td / T1
        Ki = 0
        Kd = 0
    elif tipo.upper() == 'PI':
        Kp = 0.9 * (T1 / (K * Td))
        Ti = 3 * Td
        Ki = Kp / Ti
        Kd = 0
    elif tipo.upper() == 'PID':
        Kp = 1.2 * (T1 / (K * Td))
        Ti = 2 * Td
        Td_pid = 0.5 * Td
        Ki = Kp / Ti
        Kd = Kp * Td_pid
    else:
        raise ValueError("Tipo de controlador inválido. Use 'P', 'PI' ou 'PID'.")

    return Kp, Ki, Kd

def criar_controlador(tipo, Kp, Ki, Kd):
    """
    Cria a função de transferência do controlador.

    Parameters:
    - tipo: Tipo de controlador ('P', 'PI', 'PID').
    - Kp, Ki, Kd: Parâmetros do controlador.

    Returns:
    - controlador: Função de transferência.
    """
    if tipo.upper() == 'P':
        return ctrl.TransferFunction([Kp], [1])
    elif tipo.upper() == 'PI':
        return ctrl.TransferFunction([Kp, Ki], [1, 0])
    elif tipo.upper() == 'PID':
        return ctrl.TransferFunction([Kd, Kp, Ki], [0.0001 ,1, 0])
    else:
        raise ValueError("Tipo de controlador inválido. Use 'P', 'PI' ou 'PID'.")

def transformar_controlador_z(controlador, T):
    """
    Converte o controlador em tempo discreto usando a transformada Z.

    Parameters:
    - controlador: Função de transferência do controlador em tempo contínuo.
    - T: Período de amostragem.

    Returns:
    - controlador_z: Função de transferência em tempo discreto.
    """
    return ctrl.sample_system(controlador, T, method='tustin')

def plotar_respostas(time, response, time_cl, response_cl, Td, T1, K, time_inflexion, y_inflexion, slope_at_inflexion, tipo):
    """
    Plota as respostas ao degrau do sistema original e controlado com maior clareza.
    """
    # Ajuste da tangente para começar exatamente no ponto de inflexão
    t_tangent = np.linspace(time_inflexion, time[-1], 100)  # Tangente parte do ponto de inflexão
    y_tangent = slope_at_inflexion * (t_tangent - time_inflexion) + y_inflexion

    plt.figure(figsize=(10, 6))
    plt.plot(time, response, label="Resposta Original", linewidth=2)
    plt.plot(t_tangent, y_tangent, '--', label="Tangente", linewidth=1.5, color='green')
    plt.scatter([time_inflexion], [y_inflexion], color='red', label="Ponto de Inflexão", zorder=5)
    plt.axvline(Td, color='purple', linestyle='--', label=f"T_d = {Td:.2f}")
    plt.scatter([time_inflexion + T1], [K], color='orange', label=f"T_1 = {T1:.2f}", zorder=5)
    plt.axhline(K, color='brown', linestyle='--', label=f"K = {K:.2f}")
    plt.title("Resposta ao Degrau - Sistema Original", fontsize=14)
    plt.xlabel("Tempo (s)", fontsize=12)
    plt.ylabel("Amplitude", fontsize=12)
    plt.legend(fontsize=10)
    plt.grid(True)
    plt.show()

    plt.figure(figsize=(10, 6))
    plt.plot(time_cl, response_cl, label=f"Resposta Controlada ({tipo.upper()})", linewidth=2, color='blue')
    plt.title("Resposta ao Degrau - Sistema Controlado", fontsize=14)
    plt.xlabel("Tempo (s)", fontsize=12)
    plt.ylabel("Amplitude", fontsize=12)
    plt.legend(fontsize=10)
    plt.grid(True)
    plt.show()


def exibir_informacoes(planta, controlador, sistema_fechado, Td, T1, K, tipo_controlador, Kp, Ki, Kd, controlador_z):
    """
    Exibe as informações sobre a planta, controlador, feedback e parâmetros de Ziegler-Nichols.

    Parameters:
    - planta: Função de transferência da planta.
    - controlador: Função de transferência do controlador.
    - sistema_fechado: Função de transferência do sistema em malha fechada.
    - Td, T1, K: Parâmetros calculados pelo método de Ziegler-Nichols.
    - tipo_controlador: Tipo de controlador ('P', 'PI', 'PID').
    - Kp, Ki, Kd: Parâmetros do controlador.
    - controlador_z: Controlador em tempo discreto.
    """
    print("=== Informações sobre a Planta ===")
    print(f"Planta (Função de Transferência):\n{planta}\n")

    print("=== Parâmetros de Ziegler-Nichols ===")
    print(f"T_d (Tempo de Atraso): {Td:.4f}")
    print(f"T_1 (Constante de Tempo): {T1:.4f}")
    print(f"K (Ganho da Planta): {K:.4f}\n")

    print("=== Controlador Selecionado ===")
    print(f"Tipo de Controlador: {tipo_controlador}")
    print(f"Kp (Ganho Proporcional): {Kp:.4f}")
    print(f"Ki (Ganho Integral): {Ki:.4f}")
    print(f"Kd (Ganho Derivativo): {Kd:.4f}")
    print(f"Controlador (Função de Transferência):\n{controlador}\n")

    print("=== Controlador em Tempo Discreto ===")
    print(f"Controlador (Transformada Z):\n{controlador_z}\n")

    print("=== Sistema em Malha Fechada ===")
    print(f"Sistema em Malha Fechada (Função de Transferência):\n{sistema_fechado}\n")

def main():
    try:
        tipo_controlador = input("Escolha o tipo de controlador (P, PI, PID): ").strip().upper()
        if tipo_controlador not in ['P', 'PI', 'PID']:
            raise ValueError("Tipo de controlador inválido.")

        num = [2]
        den = [1, 3, 5, 6]
        plant = ctrl.TransferFunction(num, den)

        time, response = ctrl.step_response(plant)

        Td, T1, K, slope_at_inflexion, y_inflexion = calcular_parametros_ziegler_nichols_auto(response, time)

        Kp, Ki, Kd = calcular_parametros_controlador(tipo_controlador, Td, T1, K)
        controlador = criar_controlador(tipo_controlador, Kp, Ki, Kd)

        T = 0.1  # Período de amostragem
        controlador_z = transformar_controlador_z(controlador, T)

        closed_loop_system = ctrl.feedback(controlador * plant)
        time_cl, response_cl = ctrl.step_response(closed_loop_system)

        exibir_informacoes(plant, controlador, closed_loop_system, Td, T1, K, tipo_controlador, Kp, Ki, Kd, controlador_z)

        plotar_respostas(time, response, time_cl, response_cl, Td, T1, K, Td, y_inflexion, slope_at_inflexion, tipo_controlador)

    except Exception as e:
        print(f"Erro: {e}")

if __name__ == "__main__":
    main()
