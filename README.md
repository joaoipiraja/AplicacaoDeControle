# Projetos Práticos de Aplicações e Controle de Sistemas

Este repositório contém os projetos práticos que desenvolvi para a disciplina de **Aplicações e Controle de Sistemas**. O objetivo desses projetos é projetar e implementar filtros digitais e aplicações de controle, utilizando principalmente a linguagem C para microcontroladores. Cada projeto explora diferentes técnicas de filtragem e mecanismos de controle úteis em sistemas embarcados.

## Projetos

1. **Filtro Média Móvel**: Um filtro digital que calcula a média das amostras mais recentes para suavizar o ruído do sinal.
2. **Filtro Mediana**: Um filtro digital que seleciona o valor mediano de um conjunto de amostras, útil para remover ruídos pontuais.
3. **Laboratório Proteus**: Contém arquivos e simulações desenvolvidos no software Proteus para diversos experimentos de controle e aplicação.

## Configuração e Requisitos

Os projetos são escritos em C e projetados para rodar em microcontroladores com ADC de 8 bits, especificamente utilizando o **PIC16F874**. O ambiente de desenvolvimento que usei inclui:

- **Software Proteus**: Para simulação de circuitos eletrônicos.
- **MPLAB X IDE**: Para escrever e compilar o código em C.
- **Microcontrolador PIC16F874**.
- **Configuração do ADC**: Resolução de 8 bits.

## Detalhes dos Projetos

### Filtro Média Móvel

#### Visão Geral
O Filtro Média Móvel calcula a média das últimas cinco amostras de entrada do ADC para suavizar variações nos dados do sinal. Esse filtro é comumente usado em aplicações que necessitam de redução de ruído em leituras de sensores.

#### Como Funciona
- **Amostragem do ADC**: Leio cinco valores do canal do ADC.
- **Cálculo da Média**: Calculo a média aritmética dessas amostras.
- **Saída**: Envio o resultado da média para um pino digital.

### Filtro Mediana

#### Visão Geral
O Filtro Mediana calcula a mediana de três amostras consecutivas, reduzindo o impacto de valores atípicos. Esse filtro é eficaz em aplicações onde desvios ocasionais podem afetar a precisão dos dados.

#### Como Funciona
- **Amostragem do ADC**: Leio dez amostras e repito o primeiro e o último valor para manter a continuidade nas extremidades.
- **Cálculo da Mediana**: Calculo a mediana de cada conjunto de três amostras consecutivas.
- **Saída**: Envio a mediana para um pino digital.

### Laboratório Proteus

#### Visão Geral
No **Laboratório Proteus**, incluí arquivos de simulação de circuitos desenvolvidos no software Proteus. Essas simulações servem para testar e validar os diversos métodos de controle e filtragem que implementei nos projetos.

---

Espero que esses projetos sejam úteis para quem está interessado em filtros digitais e controle de sistemas aplicados em microcontroladores. Sinta-se à vontade para explorar, utilizar e modificar o código conforme necessário.

