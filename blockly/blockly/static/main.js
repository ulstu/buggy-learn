import { Blockly, pythonGenerator, initBlockly } from './blockly.js';

document.addEventListener("DOMContentLoaded", function () {
    const workspace = initBlockly();

    console.log("Инициализация Blockly...");

    // Extract world_path from URL
    const urlParams = new URLSearchParams(window.location.search);
    const worldPath = urlParams.get('world_path');

    document.getElementById("runButton").addEventListener("click", function() {
        console.log("⚡ Нажата кнопка 'Запустить'");

        try {
            const code = pythonGenerator.workspaceToCode(workspace);
            console.log("Сгенерированный код:");
            let json_body = {code, "world_path": worldPath};
            console.log(json_body);

            fetch('/run', {
                method: 'POST',
                headers: { 'Content-Type': 'application/json' },
                body: JSON.stringify(json_body)
            })
            .then(response => response.json())
            .then(data => console.log("Ответ от сервера:", data))
            .catch(error => console.error("Ошибка при отправке кода:", error));

        } catch (error) {
            console.error("Ошибка генерации кода:", error);
        }
    });
});

