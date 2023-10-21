A pasta [mc_pos_control](src/modules/mc_pos_control/) contém classes relativas ao controle de posição, assim como a pasta [mc_att_control](src/modules/mc_att_control/) contém classes relativas ao controle de atitude.
- A pasta [src/modules/simulation/simulator_sih](src/modules/simulation/simulator_sih) contém o modelo matemático que pode ser utilizado para

Lista de atividades
- [x] desativar o controle de posição (```mc_pos_control```);
- [x] desativar o controle de atitude(```mc_att_control```);
- [x] desativar o controle de taxa de rotação(```mc_rate_control```);
- [x] incluir um módulo sdre (```mc_sdre_control```);
- [x] compilar o módulo implementado
- [x] verificar se o módulo está rodando
- [x] implementar uma função ```run()```  para o loop do nosso controle;
- [ ] ~~tirar arquivos que não são pertinentes ao nosso trabalho (como as funções de asa fixas)~~;
- [ ] ~~fazer um "fork" com uma versão limpa~~;


- Passos
  1. Desativar os controles não pertinentes [rc.mc_app](ROMFS/px4fmu_common/init.d/rc.mc_apps)
  2. Colocar a bibliotecas auxiliares na pasta [lib](src/lib/)
   3. Criar os CMakeLists.txt dentre de cada pasta[Texto sobre assunto](https://dev.px4.io/v1.10_noredirect/en/apps/hello_sky.html)
         Um exemplo pode ser visto na pasta [DRONE](src/lib/DRONE/) colocar um [CMakeLists.txt](src/lib/DRONE/CMakeLists.txt)
   4. Adicionar também uma linha no [CMakeLists.txt](src/lib/CMakeLists.txt) da pasta [lib](src/lib)




# Dúvidas e repostas
Onde é calculado a empuxo?
      No [Controle de posição](src/modules/mc_pos_control/MulticopterPositionControl.cpp#L638)

## Quais são as mínimas configurações que devem ser feitas para garantir o voo?
### Multicopter attitude control
- Durante o debug verifiquei que os módulos são executados em paralelo. Os pontos de parada ocorrem intercalados, em uma etapa ocorrem no controle de atitude enquanto que em outra ocorre no controle de taxa.
### Multicopter rate control
- Verifica se a opção sair esta ativa [`if (should_exit())`](https://github.com/roneydua/PX4-Autopilot/blob/ba12134e19f9819c0e77cb9b58bf5cfca7c75896/src/modules/mc_rate_control/MulticopterRateControl.cpp#L106)
- Mede desempenho do módulo [`perf_begin(_loop_perf)`](https://github.com/roneydua/PX4-Autopilot/blob/ba12134e19f9819c0e77cb9b58bf5cfca7c75896/src/modules/mc_rate_control/MulticopterRateControl.cpp#L112)
  - será que é obrigatório?
- Verifica se há modificações de parametros [`if (_parameter_update_sub.updated())`](https://github.com/roneydua/PX4-Autopilot/blob/ba12134e19f9819c0e77cb9b58bf5cfca7c75896/src/modules/mc_rate_control/MulticopterRateControl.cpp#L115)
- Aparentemente aqui que o controle define a velocidade do loop interno entrando quando há atualização do giroscópio. [`if (_vehicle_angular_velocity_sub.update(&angular_velocity))`](https://github.com/roneydua/PX4-Autopilot/blob/ba12134e19f9819c0e77cb9b58bf5cfca7c75896/src/modules/mc_rate_control/MulticopterRateControl.cpp#L127)
- Verifica o módo de voo [`if(_vehicle_land_detected_sub.updated())`](https://github.com/roneydua/PX4-Autopilot/blob/ba12134e19f9819c0e77cb9b58bf5cfca7c75896/src/modules/mc_rate_control/MulticopterRateControl.cpp#L142)
- Detecta se o quadrirrotor está pousado [`if (_vehicle_land_detected_sub.updated())`](https://github.com/roneydua/PX4-Autopilot/blob/ba12134e19f9819c0e77cb9b58bf5cfca7c75896/src/modules/mc_rate_control/MulticopterRateControl.cpp#L142)
- Verifica se o controle está no modo controle manual. **Só entra se o controle de atituede está ativo.** [`if (_vehicle_control_mode.flag_control_manual_enabled &&!_vehicle_control_mode.flag_control_attitude_enabled)`](https://github.com/roneydua/PX4-Autopilot/blob/ba12134e19f9819c0e77cb9b58bf5cfca7c75896/src/modules/mc_rate_control/MulticopterRateControl.cpp#L156)
