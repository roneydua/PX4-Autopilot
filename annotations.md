A pasta [mc_pos_control](src/modules/mc_pos_control/) contém classes relativas ao controle de posição, assim como a pasta [mc_att_control](src/modules/mc_att_control/) contém classes relativas ao controle de atitude.
- A pasta [src/modules/simulation/simulator_sih](src/modules/simulation/simulator_sih) contém o modelo matemático que pode ser utilizado para

Lista de atividades
- [x] desativar o controle de posição (```mc_pos_control```);
- [x] desativar o controle de atitude(```mc_att_control```);
- [x] desativar o controle de taxa de rotação(```mc_rate_control```);
- [x] incluir um módulo sdre (```mc_sdre_control```);
- [x] compilar o módulo implementado
- [x] verificar se o módulo está rodando
- [ ] implementar uma função ```run()```  para o loop do nosso controle;
- [ ] tirar arquivos que não são pertinentes ao nosso trabalho (como as funções de asa fixas);
- [ ] fazer um "fork" com uma versão limpa;


- Passos
  1. Desativar os controles não pertinentes [rc.mc_app](ROMFS/px4fmu_common/init.d/rc.mc_apps)
  2. Colocar a bibliotecas auxiliares na pasta [lib](src/lib/)
   3. Criar os CMakeLists.txt dentre de cada pasta[Texto sobre assunto](https://dev.px4.io/v1.10_noredirect/en/apps/hello_sky.html)
         Um exemplo pode ser visto na pasta [DRONE](src/lib/DRONE/) colocar um [CMakeLists.txt](src/lib/DRONE/CMakeLists.txt)
   4. Adicionar também uma linha no [CMakeLists.txt](src/lib/CMakeLists.txt) da pasta [lib](src/lib)




# Dúvidas e repostas
Onde é calculado a empuxo?
      No [Controle de posição](src/modules/mc_pos_control/MulticopterPositionControl.cpp#L638)

