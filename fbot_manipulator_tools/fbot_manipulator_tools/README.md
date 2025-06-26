# fbot_manipulator_tools

## Visão Geral

este pacote tem um nodo para salvar poses do braço WX200. O pacote permite aos usuários capturar e armazenar configurações de juntas do braço robótico para uso posterior em aplicações de manipulação.

## Estrutura do Pacote

```
fbot_manipulator_tools/
├── package.xml
├── setup.cfg
├── setup.py
├── fbot_manipulator_tools/
│   ├── __init__.py
│   └── save_wx200_arm_pose.py
└── resource/
    └── fbot_manipulator_tools
```

## Funcionalidades

### `save_wx200_arm_pose.py`

Este módulo implementa o nodo `ArmJointStateSaver` que permite salvar poses do braço WX200 em um arquivo YAML para uso posterior.

#### Características Principais:

- **Desabilitação Segura do Torque**: O nó desabilita automaticamente o torque do braço para permitir manipulação manual
- **Captura de Estados das Juntas**: Captura as posições atuais de todas as juntas do braço (excluindo o gripper)
- **Salvamento em YAML**: Armazena as poses nomeadas em formato YAML estruturado
- **Interface Interativa**: Interface de linha de comando intuitiva para nomear e salvar poses

## Como Usar

### Pré-requisitos

1. Certifique-se de que o braço WX200 esteja conectado e funcionando
2. Lance o pacote de controle do braço:
   ```bash
   ros2 launch interbotix_xsarm_control xsarm_control.launch.py robot_model:=wx200
   ```

### Executando o Nó de Salvamento de Poses

1. Execute o nó de salvamento:
   ```bash
   ros2 run fbot_manipulator_tools manipulator_saver
   ```

2. Siga as instruções na tela:
   - Confirme que o braço está na posição sleep antes de desabilitar o torque
   - Especifique o nome do arquivo YAML (ex: `arm_poses.yaml`)
   - Mova o braço manualmente para a pose desejada
   - Digite um nome para a pose (ex: 'PrePickup', 'LookToGarbage')
   - Escolha se deseja adicionar mais poses

3. O arquivo será salvo no diretório config do workspace

### Exemplo de Uso

```bash
$ ros2 run fbot_manipulator_tools manipulator_saver

The torque will be disabled. The Arm is in the sleep pose? (y/n): y
[WARN] [wx200ArmPoseSaver]: Disabling the torque
Enter the name of the file to save the joint states (e.g., arm_poses.yaml): my_poses.yaml
Move the arm to the desired pose and enter its name (e.g., 'PrePickup', 'LookToGarbage'): PrePickup
[INFO] [wx200ArmPoseSaver]: Received msg: [('waist', 0.0), ('shoulder', -1.57), ('elbow', 1.57), ('wrist_angle', 0.0), ('wrist_rotate', 0.0)]
[INFO] [wx200ArmPoseSaver]: Pose 'PrePickup' saved.
Do you want to add more poses? (y/n): n
[WARN] [wx200ArmPoseSaver]: Enabling the torque
[INFO] [wx200ArmPoseSaver]: Poses saved to my_poses.yaml. Shutting down node.
```

### Formato do Arquivo YAML

O arquivo gerado terá a seguinte estrutura:

```yaml
poses:
  PrePickup:
    waist: 0.0
    shoulder: -1.5707963267948966
    elbow: 1.5707963267948966
    wrist_angle: 0.0
    wrist_rotate: 0.0
  LookToGarbage:
    waist: 1.2
    shoulder: -0.8
    elbow: 1.2
    wrist_angle: 0.5
    wrist_rotate: 0.0
```

## Segurança

⚠️ **Aviso Importante**: 
- **Sempre certifique-se de que o braço está em uma posição segura (sleep) antes de desabilitar o torque**

- O torque será automaticamente reabilitado após salvar todas as poses o braço não ira voltar para a posição de sleep