# Инструкция по работе с многопользовательским сервером программирования роботов

## Регистрация на портале
1. Необходимо пройти регистрацию на портале https://robots.ulstu.ru и сообщить свой логин организаторам  https://t.me/hibers. Для каждой команды создается только один аккаунт.

2. Дождаться модерации и подготовки личного кабинета (до 24 часов).

## Работа с порталом

1. Для доступа к рабочему окружению необходимо перейти в меню "Турниры" и среди карточек турнира выбрать нужный турнир

2. На странице турнира для перехода в рабочее окружение конкретного соревнования необходимо нажать кнопку "Запустить задачу".

3. Откроется страница, на которой будут доступны 3 вкладки:

3.1. VS Code - редактор исходных кодов с терминалом, в котором необходимо выполнять команды по компиляции решения и его запуску.

3.2. Webots - после запуска решения на этой вкладке откроется визуализация Webots. Для доступа к визуализации необходимо после открытия вкладки нажать кнопку Connect, адрес websocket не нужно менять.

3.3. Редактор карты - страница с глобальной картой соревнований и изображением с фронтальной камеры автомобиля.

Каждая вкладка может быть открыта для удобства в отдельной вкладке браузера. Для этого необходимо нажать соответствующую кнопку около наименования вкладки.
Содержимое вкладки может быть обновлено (перезагружен iframe). Для этого необходимо нажать соответствующую кнопку около наименования вкладки.

4. Для того, чтобы судьи могли запустить решение участников, необходимо указать в соответствующем поле команду для запуска. По умолчанию это команда шаблонного решения. Команда для запуска у каждого участника может быть своя.

```bash
ros2 launch webots_ros2_suv robot_launch.py
```

## Общие замечания
1. За час до финала решения участников "замораживаются" Это значит, что команда теряет доступ к редактору исходных кодов, редактору карты и симуляции. Они становятся доступны только судьям  по закрытому для участников паролю.

2. Для загрузки своего решения вы можете в дереве проекта VS Code загрузить zip архив со своим решением (контекстное меню Upload), а затем в терминале VS Code разархивировать загруженный файл.