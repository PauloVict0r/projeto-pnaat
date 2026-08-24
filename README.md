Este é o repositório compartilhado para os projetos do PNAAT. Apenas as pastas na raíz do projeto (sample_project e hello_world) contém os arquivos relacionados ao primeiro projeto 
(sistemas embarcados e IOT), sendo hello_world o projeto para captação dos dados de treinamento do modelo de classificação e sample_project responsável por rodar o modelo localmente
e gerar as classificações

Este projeto inicial se baseia nos arquivos gerados nas aulas 8 e 9 da apostila de IOT, relacionados a  conectar o sensor BNO085 ao ESP32s3 para a coleta de dados inerciais (aula 8) e,
com base nos dados coletados, treinar um modelo de IA para classificação dos dados via Edge Impulse (aula 9).

A tarefa específica deste projeto é usar o BNO085 pra coletar as informações inerciais para que o ESP32s3 classifique o estado atual em uma de 6 labels (repouso, direita, esquerda, frente
, trás e cabeca_baixo) a depender da posição que o sensor estiver inclinado. Em testes sintéticos na plataforma da Edge Impulse o modelo alcançou 92% de precisão nas aferições, mas ocilou
bastante em ambiente real.

É importante notar que as aferições levam 3 segundos para serem feitas com precisão, apesar do monitor OLED atualizar com 50hz.
