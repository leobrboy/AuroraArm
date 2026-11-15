# Relatório Técnico: Model Based Design da Junta AuroraArm

**Instituição:** UNIFECAF  
**Curso:** Engenharia da Computação  
**Estudante:** Leonardo Ramiro Santos do Nascimento  
**Cidade:** São José do Rio Claro - MT  
**Ano:** 2026

## Resumo

A junta do cotovelo do AuroraArm precisa se mover em uma linha de montagem compartilhada com operadores. Para estudar esse problema sem depender de um robô físico, modelei a planta e os sinais do acionamento em software, usei um autômato para o supervisor de segurança e comparei a lei de controle do GNU Octave com o C compilado. O ensaio no QEMU executa o código num Cortex-M4 emulado; ele não substitui a medição de um equipamento real.

Na resposta ao degrau de posição de 0,1 rad, o sobressinal ficou em 1,66% e o tempo de acomodação em 240 ms, dentro dos limites de 8% e 300 ms. Após 30 s, o erro numérico caiu para aproximadamente 3,94e-10 rad. A análise de velocidade também revelou uma incompatibilidade importante: o ponto crítico de 450 rad/s no motor produziria 1.575 mm/s na extremidade, acima dos 250 mm/s permitidos na zona colaborativa. Por isso, o código limita a referência de velocidade nesse modo.

## 1. Caso e requisitos

O AuroraArm é um braço colaborativo fictício de seis graus de liberdade. Este estudo trata somente da junta do cotovelo. Ela usa um motor CC de ímã permanente de 48 V e uma transmissão de redução 100:1. No ponto crítico de pega e soltura, o eixo do motor deve sustentar 450 rad/s contra um torque de carga refletido de 0,25 N m. O payload informado é de 2 kg, a 0,35 m do eixo.

O enunciado já fornece o torque refletido no motor e a inércia refletida. Por isso, o payload não é convertido novamente em torque nem somado à inércia: fazê-lo contaria a mesma carga duas vezes.

| Parâmetro | Símbolo | Valor |
|---|---:|---:|
| Resistência de armadura | R | 0,8 ohm |
| Indutância de armadura | L | 1,2 mH |
| Constante de torque | Kt | 0,085 N m/A |
| Constante de f.c.e.m. | Ke | 0,085 V s/rad |
| Inércia refletida | J | 4,2e-4 kg m2 |
| Atrito viscoso | b | 1,5e-4 N m s/rad |
| Redução | N | 100:1 |
| Limites do acionamento | Vmax, imax | 48 V, 8 A |
| Período de amostragem | Ts | 2 ms (500 Hz) |
| Orçamento de execução e memória | - | <200 us/ciclo; 256 KB |

Os requisitos de controle são: erro de posição nulo em regime após degrau (CTRL-001), sobressinal de até 8% (CTRL-002), acomodação em até 300 ms (CTRL-003) e limites instantâneos de 48 V e 8 A (CTRL-004).

Os requisitos de segurança são: emitir torque zero em até 100 ms quando o scanner detectar uma mão a menos de 500 mm (SAFE-001); impedir retomada automática depois da parada (SAFE-002); retomar a trajetória após a mão sair do campo e o operador rearmar o sistema (SAFE-003); e limitar a velocidade linear do efetuador a 250 mm/s dentro da zona colaborativa (SAFE-004).

SAFE-001, SAFE-002 e SAFE-004 são propriedades de segurança: especificam condições que não podem ocorrer. SAFE-003 é uma propriedade de vivacidade condicionada: depois que o campo fica livre e o operador rearma o sistema, o movimento deve voltar a ocorrer. Os requisitos de controle medem desempenho da malha; os de segurança limitam as ações do supervisor.

## 2. Modelo da junta

Usei três estados: corrente de armadura, velocidade angular do motor e posição angular do motor, isto é, x=[i, omega_m, theta_m]^T. A corrente determina o torque eletromagnético; a velocidade e a posição descrevem o movimento. A tensão de armadura é a entrada. A saída de posição é a posição da junta, theta_j=theta_m/N.

As equações vêm do balanço elétrico e mecânico do motor:

- L di/dt = Va - R i - Ke omega_m
- J d(omega_m)/dt = Kt i - b omega_m - tau_L
- d(theta_m)/dt = omega_m; theta_j = theta_m/N

O torque constante tau_L entra como perturbação. Linearizando em torno do ponto de operação, obtém-se:

    A = [-R/L, -Ke/L, 0; Kt/J, -b/J, 0; 0, 1, 0]
    B = [1/L; 0; 0]
    Cj = [0, 0, 1/N], D = 0

Com os parâmetros do enunciado:

    A = [-666.6667, -70.8333, 0;
          202.3810, -0.357143, 0;
            0,       1,       0]
    B = [833.3333; 0; 0]

Os autovalores são 0, -22.6152 e -644.4086 s^-1. O polo na origem aparece porque a velocidade é integrada para obter posição. A constante de tempo elétrica simples vale tau_e=L/R=1,5 ms. Para a dinâmica mecânica, incluindo o amortecimento elétrico refletido, tau_m=J/(b+Kt x Ke/R)=45,745 ms. A razão tau_m/tau_e é 30,50.

Com Ts=2 ms, há 22,87 amostras por constante de tempo mecânica. Isso dá resolução adequada à dinâmica mecânica dominante. Já a constante elétrica é menor que o período de amostragem: por isso, usei ZOH exato na simulação, em vez de Euler explícito. No equipamento, a malha de corrente deve ser mais rápida ou validada com cuidado.

As matrizes de controlabilidade e observabilidade têm posto 3. Para observabilidade, usei Cj, que mede a posição da junta. A escala 1/N não muda o posto.

## 3. Ponto de operação

Em regime permanente, a aceleração é zero. A corrente necessária para equilibrar o torque é:

    i_op = tau_L/Kt = 0,25/0,085 = 2,9412 A

A tensão precisa vencer a queda resistiva e a força contraeletromotriz:

    V_op = R x i_op + Ke x omega_m
         = 0,8 x 2,9412 + 0,085 x 450 = 40,6029 V

Restam 5,0588 A até o limite de corrente e 7,3971 V de margem positiva de tensão. O ponto crítico é alcançável pelos valores nominais, mas a margem de tensão é curta para aceleração adicional. O controlador precisa tratar a saturação. O integrador usa anti-windup para não continuar acumulando erro enquanto o atuador está limitado.

## 4. Controlador e resposta ao degrau

Implementei um PID discreto com derivada da posição medida, saturação e integração condicional. O ensaio usa um degrau de posição de 0,1 rad na junta, carga constante de 0,25 N m, amostragem de 2 ms e planta discretizada por ZOH exato. A corrente é limitada a 8 A por previsão do valor no próximo passo. Os ganhos usados foram Kp=800, Ki=400 e Kd=45, nas unidades compatíveis com erro em radianos, derivada em rad/s e saída em volts.

| Métrica para degrau de 0,1 rad | Resultado | Limite | Avaliação |
|---|---:|---:|---|
| Sobressinal | 1,66% | <=8% | Dentro do limite |
| Acomodação, banda de +/-2% | 240 ms | <=300 ms | Dentro do limite |
| Pico de tensão no modelo | 14,41 V | <=48 V | Dentro do limite |
| Pico de corrente no modelo | 8,00 A | <=8 A | No limite |
| Erro numérico aos 30 s | 3,94e-10 rad | Zero em regime | Abaixo da tolerância numérica de 1e-4 rad |

A ação integral remove, em regime, o erro provocado por uma carga constante, desde que o sistema estabilize e não permaneça saturado. Para avaliar CTRL-001, adotei uma tolerância numérica explícita de 1e-4 rad, pois uma simulação em ponto flutuante não representa zero matemático exato. O erro após 30 s ficou muito abaixo dela. O sobressinal e a acomodação também ficam dentro dos limites para o degrau adotado. A corrente chega a 8 A, sem margem para incerteza de sensor ou de modelo.

Também rodei o mesmo degrau num modelo de software mais detalhado. Ele quantiza a posição de um encoder hipotético de 65.536 contagens efetivas por volta do motor, a leitura de corrente por um ADC de 12 bits em +/-8 A e o comando do acionamento por 12 bits em +/-48 V. Esses valores de resolução são hipóteses do ensaio, pois o enunciado não especifica componentes. Variei o torque de carga entre 0,20 e 0,30 N m e incluí um caso com um ciclo de atraso do comando, equivalente a 2 ms.

| Cenário simulado | Sobressinal | Acomodação | Erro final | Pico de tensão / corrente |
|---|---:|---:|---:|---:|
| Carga 0,20 N m | 0,96% | 258 ms | 2,23e-5 rad | 14,21 V / 7,91 A |
| Carga 0,25 N m | 0,90% | 260 ms | 7,51e-7 rad | 14,16 V / 7,92 A |
| Carga 0,30 N m | 0,56% | 266 ms | 2,08e-7 rad | 14,18 V / 7,92 A |
| Carga 0,25 N m e atraso de 2 ms | 0,87% | 262 ms | 1,71e-6 rad | 14,14 V / 7,92 A |

As quatro simulações ficam dentro dos limites de sobressinal, acomodação, erro, tensão e corrente adotados. Para o cenário com atraso, o modelo prevê o efeito do comando que já está na fila antes de escolher a próxima tensão. O modelo ainda não inclui folga mecânica, flexibilidade do braço, ruído aleatório, aquecimento, dinâmica do freio nem escalonamento de tempo real. Os picos de cada execução ficam em `tests/software_only_results.txt`.

## 5. Supervisor e propriedades formais

Modelei o supervisor com cinco estados: MOVING, STOPPING, STOPPED, FIELD_CLEAR e REARMED. Os eventos são detecção de obstáculo, confirmação de torque zero, campo livre, rearme do operador e comando de retomada. O caminho permitido de volta ao movimento é:

    MOVING -> STOPPING -> STOPPED -> FIELD_CLEAR -> REARMED -> MOVING

Os últimos três passos exigem, nessa ordem, campo livre, rearme e comando de retomada. STOPPED não tem transição direta para MOVING.

Verifiquei duas propriedades em notação LTL. A propriedade de segurança é `G((STOPPED & !field_clear & !operator_rearm) -> X(!MOVING))`: enquanto não houver campo livre e rearme, o próximo estado não pode ser MOVING. A propriedade de vivacidade condicional é `G((STOPPED & field_clear & X(operator_rearm & X(resume))) -> XXX(MOVING))`: dada a sequência autorizadora, o movimento volta após três transições.

O script enumera todas as 3.125 sequências de cinco eventos possíveis a partir de MOVING. Foram cobertos os cinco estados e os 25 pares estado-evento. A propriedade de segurança não encontrou violação no modelo corrigido, e a busca encontrou o caminho de vivacidade quando os eventos requeridos são apresentados. Isso pressupõe que o scanner, o sinal de campo livre e o rearme estejam corretos. A posição e a trajetória original não fazem parte deste autômato.

Durante a modelagem, considerei uma versão preliminar que aceitava `resume` no estado STOPPED. O contraexemplo é curto: `obstacle -> torque_zero_confirmed -> resume`. Ele leva a MOVING sem campo livre nem rearme. A transição foi removida; FIELD_CLEAR e REARMED agora são estados separados.

Para SAFE-001, varri atrasos do scanner de 0 a 20 ms em passos de 1 ms, jitter de 0 ou 2 ms e um ciclo de resposta do supervisor de 2 ms. O pior orçamento lógico é 20+2+2=24 ms, menor que o prazo de 100 ms. Esse resultado cobre a emissão do comando de torque zero depois da detecção. Não mede a desaceleração física nem a distância de parada. Para isso, seriam necessários o modelo do freio, a carga, o perfil de parada e ensaios do conjunto real.

Para SAFE-004, a velocidade linear da extremidade é v=omega_m x r/N. No ponto crítico, v=450 x 0,35/100=1,575 m/s, ou 1.575 mm/s. O limite seguro de referência é omega_m<=0,250 x 100/0,35=71,43 rad/s no modo colaborativo. O C gerado limita a referência e o harness testa uma solicitação de 450 rad/s. Isso não garante, sozinho, que a velocidade física nunca exceda 250 mm/s durante transientes; o monitor de segurança e a resposta do acionamento ainda precisam ser validados no equipamento. O ponto de 450 rad/s deve ocorrer fora da zona colaborativa ou com separação física apropriada.

## 6. C gerado, SIL e hardware

O script `auroraarm.m` gera `generated/auroraarm_controller.c`. O C contém saturação de tensão, limitação de corrente preditiva para uma amostra, integração condicional e limitador da referência de velocidade colaborativa. A previsão elétrica usa a corrente medida e supõe velocidade constante durante o passo; essa simplificação precisa ser revista para o circuito real.

Para o SIL, compilei o C gerado com GCC e executei o harness em 10.001 vetores, iguais aos usados pelo modelo de referência do Octave. A maior diferença entre as saídas foi 1,2511e-5 V. O critério de aceitação aplicado foi 1e-4 V. A diferença supera epsilon de `float` (1,1921e-7), como se espera quando operações arredondadas se acumulam, mas fica abaixo da tolerância de saída definida. O ensaio compara a lei de controle compilada; não qualifica o gerador de código.

Também instalei QEMU 11.1 e o Arm GNU Embedded Toolchain 14.2.Rel1 no computador de desenvolvimento. O firmware de teste foi compilado para a máquina `mps2-an386`, que representa um Cortex-M4, e executado no emulador. Ele aplica 2.001 vetores de referência à rotina C gerada. O resultado foi `M4_QEMU_PASS`, com diferença máxima de 10 uV, limitador de velocidade aprovado e nenhuma violação do limite de tensão. A seção `text` do ELF de teste ocupa 52.716 bytes.

Esse ensaio foi compilado com ABI soft-float. Ele verifica a execução do código ARM e o comportamento da rotina de controle no modelo Cortex-M4, mas não valida instruções de FPU. O harness também mediu 0,018 us por chamada em média no processador do computador, ao longo de um milhão de chamadas; o objeto host tem 588 bytes na seção `text`. Nenhum desses números é WCET do Cortex-M4. A imagem final de firmware também não está ligada contra os periféricos e tarefas do robô. Para demonstrar WCET abaixo de 200 us, memória total abaixo de 256 KB e uso da FPU, ainda é preciso configurar o alvo específico e medir nele, ou usar um modelo que implemente essa configuração de FPU.

Para a validação exclusivamente por software, `tests/software_only_scenarios.m` fecha a malha entre controlador, planta elétrica e mecânica, leitura quantizada do encoder e da corrente, comando de tensão quantizado e carga variável. O ensaio inclui o atraso de um ciclo de comando com uma previsão de corrente que considera o comando pendente. Junto com a enumeração do supervisor, os cenários cobrem a resposta nominal e variações definidas de carga e atraso. Os resultados são reproduzíveis no computador e servem como evidência do modelo, não como medida de parada física, ruído real ou execução em tempo real.

## 7. Rastreabilidade

| Requisito | Modelo ou elemento | Código/artefato | Evidência e estado |
|---|---|---|---|
| CTRL-001 | PID com integral | `auroraarm.m` | 3,94e-10 rad aos 30 s; abaixo da tolerância adotada de 1e-4 rad |
| CTRL-002 | Resposta de posição | `figures/position_step.png`, `tests/software_only_results.txt` | 1,66% no ZOH; até 0,97% nos cenários quantizados |
| CTRL-003 | Banda de +/-2% | `results.txt`, `tests/software_only_results.txt` | 240 ms no ZOH; até 266 ms nos cenários quantizados |
| CTRL-004 | Saturação de tensão e corrente | C gerado e planta ZOH, `tests/software_only_results.txt` | Picos por cenário registrados; modelo respeita os limites nominais |
| SAFE-001 | Atraso do scanner, jitter e ciclo do supervisor | `auroraarm.m` | 24 ms lógicos; parada física não demonstrada |
| SAFE-002 | Sem transição STOPPED para MOVING | Autômato | 3.125 sequências; 5/5 estados e 25/25 pares cobertos |
| SAFE-003 | Campo livre, rearme e retomada | Autômato | Caminho condicional encontrado; trajetória não simulada |
| SAFE-004 | Velocidade na extremidade | C gerado e relação cinemática | 1.575 mm/s no ponto crítico; referência segura limitada a 71,43 rad/s |
| HW-001 | Ts=2 ms e WCET <200 us | `tests/host_timing.txt` | Média no host de 0,018 us; falta medição de WCET no Cortex-M4 |
| HW-002 | Memória <=256 KB | ELF M4 de teste | 52.716 bytes na seção `text`; falta imagem final de firmware/periféricos |

## 8. Referências

NEWCOMBE, Chris et al. How Amazon Web Services Uses Formal Methods. *Communications of the ACM*, v. 58, n. 4, 2015. Material fornecido na disciplina.

WESCOTT, Tim. PID Without a PhD. *Embedded Systems Programming*. Material fornecido na disciplina.

INTERNATIONAL ORGANIZATION FOR STANDARDIZATION. *ISO/TS 15066: Robots and robotic devices - Collaborative robots*. Geneva: ISO, 2016.

SICILIANO, Bruno et al. *Robotics: Modelling, Planning and Control*. London: Springer, 2010.

INTERNATIONAL ELECTROTECHNICAL COMMISSION. *IEC 61508: Functional safety of electrical/electronic/programmable electronic safety-related systems*. Geneva: IEC, 2010.

BAIER, Christel; KATOEN, Joost-Pieter. *Principles of Model Checking*. Cambridge: MIT Press, 2008.

ABB. *Single-arm YuMi cobot speeds up assembly at GE Healthcare's prototype line*. ABB News, 2021. Disponível em: https://www.abb.com/global/en/news/80931/cstmr-single-arm-yumi-cobot-speeds-up-assembly-at-ge-healthcares-prototype-line. Acesso em: 15 nov. 2026. O relato serve de contexto para integração de cobots em linhas de montagem; não é evidência de segurança para o AuroraArm.

QEMU PROJECT. *QEMU System Emulation User's Guide: Arm MPS2 and MPS3 boards*. Disponível em: https://www.qemu.org/docs/master/system/arm/mps2.html. Acesso em: 15 nov. 2026. A máquina `mps2-an386` é documentada como implementação Cortex-M4.

ARM. *Arm GNU Toolchain*. Disponível em: https://developer.arm.com/Tools%20and%20Software/GNU%20Toolchain. Acesso em: 15 nov. 2026.

## 9. Reprodutibilidade

Para reproduzir a análise completa por software, abra o GNU Octave na pasta do projeto e execute `run('auroraarm.m')`. O script atualiza `results.txt`, gera as figuras, o C e os vetores de referência, executa o SIL no host e roda os cenários quantizados em `tests/software_only_scenarios.m`. Para repetir a execução do ELF no Cortex-M4 emulado, execute `powershell -NoProfile -ExecutionPolicy Bypass -File tests/run_m4_qemu.ps1`. O código e o relatório técnico estão no repositório https://github.com/leobrboy/AuroraArm.
