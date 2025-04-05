import * as Blockly from 'blockly';
import { pythonGenerator } from 'blockly/python';

// ✅ Определение блоков
Blockly.defineBlocksWithJsonArray([
    {
        "type": "move_forward",
        "message0": "Двигаться вперёд",
        "previousStatement": null,
        "nextStatement": null,
        "colour": 160,
        "tooltip": "Движение робота вперёд"
    },
    {
        "type": "turn_left",
        "message0": "Повернуть влево",
        "previousStatement": null,
        "nextStatement": null,
        "colour": 120,
        "tooltip": "Поворот робота влево"
    },
    {
        "type": "turn_right",
        "message0": "Повернуть вправо",
        "previousStatement": null,
        "nextStatement": null,
        "colour": 240,
        "tooltip": "Поворот робота вправо"
    }
]);

console.log("✅ Кастомные блоки зарегистрированы в Blockly");

// ✅ Определение генераторов JavaScript
pythonGenerator.forBlock['move_forward'] = function(block) {
    return "move_forward()\n";
};

pythonGenerator.forBlock['turn_left'] = function(block) {
    return "turn_left()\n";
};

pythonGenerator.forBlock['turn_right'] = function(block) {
    return "turn_right()\n";
};

console.log("✅ Генераторы JavaScript зарегистрированы");

// ✅ Функция для инициализации Blockly
function initBlockly() {
    console.log("🚀 Запуск Blockly...");

    const toolbox = {
        "kind": "flyoutToolbox",
        "contents": [
            { "kind": "block", "type": "move_forward" },
            { "kind": "block", "type": "turn_left" },
            { "kind": "block", "type": "turn_right" },
            { "kind": "block", "type": "controls_repeat" },
            { "kind": "block", "type": "math_number" }
        ]
    };

    const workspace = Blockly.inject('blocklyDiv', { toolbox });

    console.log("✅ Blockly успешно инициализирован.");
    return workspace;
}

// ✅ Экспорт функций
export { Blockly, pythonGenerator, initBlockly };
