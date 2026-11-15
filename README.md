# AuroraArm — projeto baseado em modelos (Model-Based Design)

Estudo acadêmico da junta do cotovelo do AuroraArm, implementado com GNU Octave. Este projeto não é um sistema de segurança certificado e não autoriza a operação de um robô colaborativo real.

## Como executar

Abra o GNU Octave nesta pasta e execute:

```
run('auroraarm.m')
```

O script calcula o modelo contínuo em espaço de estados, autovalores, postos de controlabilidade e observabilidade, constantes de tempo, ponto de operação, resposta ao degrau com ZOH exato, enumeração do supervisor e análise de temporização. Também gera `generated/auroraarm_controller.c`, os vetores de teste SIL em C e um subconjunto para o emulador Cortex-M4. O script compila e executa o harness SIL no computador e os cenários de robustez por software. São atualizados `results.txt`, `tests/host_timing.txt`, `tests/software_only_results.txt`, `figures/headroom.png`, `figures/position_step.png` e `figures/software_only_step.png`.

## Emulação do Cortex-M4

Para executar o firmware de teste, instale o QEMU 11.1 e o Arm GNU Embedded Toolchain 14.2.Rel1. O script do PowerShell procura os executáveis no `PATH`. Se necessário, defina `ARM_NONE_EABI_GCC` e `QEMU_SYSTEM_ARM` com os caminhos dos executáveis. O script também verifica os diretórios padrão de instalação no Windows.

Depois de executar o modelo no Octave para gerar o cabeçalho com os vetores, compile o controlador e execute o teste na máquina `mps2-an386` do QEMU:

```
powershell -NoProfile -ExecutionPolicy Bypass -File tests/run_m4_qemu.ps1
```

O teste executa 2.001 vetores no controlador compilado para Cortex-M4, verifica os limitadores de tensão e de referência de velocidade colaborativa e compara a saída com o Octave, com tolerância de 1e-4 V. O firmware usa ABI soft-float. Assim, o ensaio verifica a execução das instruções M4 e a lógica do controlador, mas não testa instruções da FPU, WCET no alvo, temporização real dos periféricos ou a dinâmica física do robô.

## Premissas do modelo

- A velocidade especificada para o motor se refere ao eixo do motor; a velocidade da junta é a velocidade do motor dividida pela redução 100:1.
- O torque de carga refletido de 0,25 N m e a inércia refletida de 4,2e-4 kg m² são usados diretamente. Os dados do payload não são somados outra vez.
- O ensaio de desempenho usa um degrau de posição de 0,1 rad na junta, carga constante, amostragem de 2 ms e banda de acomodação de +/-2%.
- Os cenários de robustez por software supõem um encoder com 65.536 contagens efetivas por volta, ADC de corrente de 12 bits na faixa de +/-8 A e comando de tensão assinado de 12 bits na faixa de +/-48 V. São hipóteses de simulação, não especificações de componentes identificados.
- A variação de carga e o atraso de um ciclo do comando são simulados com um limitador preditivo de corrente que considera o comando pendente. Ruído aleatório, folga, elos flexíveis, efeitos térmicos, dinâmica do freio e escalonamento de tempo real do alvo não fazem parte do modelo.
- A verificação do supervisor é de estados finitos e supõe que os eventos do scanner, do campo livre e do rearme do operador estejam corretos. Ela verifica o prazo lógico para emitir o comando de torque zero, não a distância física de parada.
- O limitador de corrente do código gerado prevê um passo elétrico e supõe velocidade medida constante durante esse intervalo. Um equipamento real exigiria análise própria do acionamento e validação dos tempos no alvo.

## Resultados e limites conhecidos

O ponto de operação crítico exige 40,603 V e 2,941 A. No modelo ZOH, o degrau de 0,1 rad apresenta sobressinal de 1,66% e acomodação em 240 ms. Após 30 s, o erro de posição simulado é 3,94e-10 rad, abaixo da tolerância numérica declarada de 1e-4 rad. A mesma velocidade crítica do motor produz 1.575 mm/s no raio informado de 0,35 m, portanto não pode ser permitida dentro da zona colaborativa com limite de 250 mm/s. O limitador de referência de velocidade segura está incluído e foi exercitado no harness compilado.

A diferença máxima da saída SIL entre o C e o modelo é comparada com 1e-4 V. O tempo e o tamanho do objeto foram medidos no computador de desenvolvimento; o WCET no Cortex-M4 (\<200 us) e o tamanho da imagem completa do firmware (\<=256 KB) ainda dependem da configuração específica do alvo.

## Repositório remoto

https://github.com/leobrboy/AuroraArm
