
# Среда для подготовки программных решений для соревнований по робототехнике в среде симуляции ROS2

## Требования к компьютеру

* OS Windows версии >= 10 или Linux Ubuntu версии >= 20
* Доступное файловое хранилище не менее 40Гб
* ОЗУ не менее 8Гб
* GPU NVidia серии >= 1060
* CPU Intel i5 и выше
* Актуальные версии  docker и драйверов nvidia

Ниже описаны инструкции для установки в Windows и Linux Ubuntu в среде docker


## Установка в среде Docker в Linux

1. Установите `git`, `make`, `curl`, `docker`, `nvidia-container-toolkit`, `nvidia-docker2`

```sh
sudo apt install git make curl
curl -sSL https://get.docker.com | sh && sudo usermod -aG docker $USER
```

   Docker желательно поставить в режиме, чтобы команды docker были доступны для запуска [без sudo](https://docs.docker.com/engine/install/linux-postinstall/).

   Перезагрузите компьютер.
   
   Установите [nvidia-container-toolkit](https://docs.nvidia.com/datacenter/cloud-native/container-toolkit/latest/install-guide.html) для обработки данных в GPU.

2. Запустите сценарий подготовки, чтобы собрать образ и запустить контейнер:

```sh
make build
```
   
Дождитесь завершения работы скрипта (около 1 часа)

3. Настройка и запуск
```sh
make all
```

4. Доступ к терминалу запущенного контейнера возможен выполнением команды
   ```sh
   docker exec -it ulstub-${FLAVOR} bash
   ```
   Вместо ${FlAVOR} пропишите название контейнера, которое было прописано в скрипте start.sh (по умолчанию devel)


## Установка в среде Docker в Windows

1. Установите [git](https://git-scm.com/download/win), [docker](https://docs.docker.com/desktop/install/windows-install/), [wsl2](https://www.solveyourtech.com/how-to-install-wsl2-on-windows-11-a-step-by-step-guide-for-beginners/) и внутри wsl установите [nvidia-container-toolkit](https://docs.nvidia.com/datacenter/cloud-native/container-toolkit/latest/install-guide.html#installing-with-apt)
2. Склонируйте репозиторий в выбранную папку на компьютере 
```bash
git clone https://github.com/ulstu/buggy-learn.git
```

3. В терминале или в среде powershell зайдите в папку проекта
```bash
cd buggy-learn
```

4. Выполните команду для построения образа
```bash
.\run_windows.bat build
```
Команда выполняется около одного часа. Наберитесь терпения.
Если в процессе установки возникнет проблема, то перезапустите Docker engine и заново выполните скрипт.

5. Создайте docker контейнер командой (по умолчанию имя контейнера ulstu-devel)
```bash
.\run_windows.bat run
```

6. Создайте символические ссылки для рабочих директорий в папке ~/ros2_ws в контейнере командой
```bash
.\run_windows.bat setup-environment
```

Если Вы установили Docker в связке с WSL, то нужно еще выполнить команду
```bash
.\run_windows.bat wsl-fix
```

7. Запустите Webots командой 
```bash
.\run_windows.bat start-webots
```

8. Запустите web-сервер разработки командой 
```bash
.\run_windows.bat start-dev-server
```


## Запуск среды разработки
1. Доступ к Редактору программ Blockly: http://localhost:31415

## Установка через docker в MacOS m1 не поддерживается, т.к. отсутствует linux дистрибутив webots arm
https://everythingdevops.dev/building-x86-images-on-an-apple-m1-chip/