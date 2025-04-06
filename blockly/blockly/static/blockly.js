import * as Blockly from 'blockly';
import { pythonGenerator } from 'blockly/python';

// Debug log to confirm Blockly.Xml.textToDom is available
console.log("Blockly.Xml.textToDom:", typeof Blockly.Xml.textToDom);

// ✅ Определение кастомных блоков
Blockly.defineBlocksWithJsonArray([
    {
        "type": "move_forward",
        "message0": "Двигаться вперёд на %1 мс",
        "args0": [
            {
                "type": "input_value",
                "name": "DURATION",
                "check": "Number"
            }
        ],
        "previousStatement": null,
        "nextStatement": null,
        "colour": 160,
        "tooltip": "Движение робота вперёд на заданную длительность"
    },
    {
        "type": "move_backward",
        "message0": "Двигаться назад на %1 мс",
        "args0": [
            {
                "type": "input_value",
                "name": "DURATION",
                "check": "Number"
            }
        ],
        "previousStatement": null,
        "nextStatement": null,
        "colour": 180,
        "tooltip": "Движение робота назад на заданную длительность"
    },
    {
        "type": "turn_left",
        "message0": "Повернуть влево на %1 мс",
        "args0": [
            {
                "type": "input_value",
                "name": "DURATION",
                "check": "Number"
            }
        ],
        "previousStatement": null,
        "nextStatement": null,
        "colour": 120,
        "tooltip": "Поворот робота влево"
    },
    {
        "type": "turn_right",
        "message0": "Повернуть вправо на %1 мс",
        "args0": [
            {
                "type": "input_value",
                "name": "DURATION",
                "check": "Number"
            }
        ],
        "previousStatement": null,
        "nextStatement": null,
        "colour": 240,
        "tooltip": "Поворот робота вправо"
    }
]);

console.log("✅ Кастомные блоки зарегистрированы в Blockly");

// ✅ Генераторы Python для кастомных блоков
pythonGenerator.forBlock['move_forward'] = function(block) {
    const duration = pythonGenerator.valueToCode(block, 'DURATION', pythonGenerator.ORDER_NONE) || '1000';
    return `MoveForward(${duration})\n`;
};

pythonGenerator.forBlock['move_backward'] = function(block) {
    const duration = pythonGenerator.valueToCode(block, 'DURATION', pythonGenerator.ORDER_NONE) || '1000';
    return `MoveBackward(${duration})\n`;
};

pythonGenerator.forBlock['turn_left'] = function(block) {
    const duration = pythonGenerator.valueToCode(block, 'DURATION', pythonGenerator.ORDER_NONE) || '1000';
    return `TurnLeft(${duration})\n`;
};

pythonGenerator.forBlock['turn_right'] = function(block) {
    const duration = pythonGenerator.valueToCode(block, 'DURATION', pythonGenerator.ORDER_NONE) || '1000';
    return `TurnRight(${duration})\n`;
};

console.log("✅ Генераторы Python зарегистрированы");

// ✅ Функция для инициализации Blockly с расширенным набором инструментов
function initBlockly() {
    console.log("🚀 Запуск Blockly...");

    const toolbox = {
        "kind": "flyoutToolbox",
        "contents": [
            {"kind": "block", "type": "move_forward"},
            {"kind": "block", "type": "move_backward"},
            {"kind": "block", "type": "turn_left"},
            {"kind": "block", "type": "turn_right"},
            {"kind": "block", "type": "controls_if", "label": "ЕСЛИ"},
            {"kind": "block", "type": "logic_compare", "label": "СРАВНЕНИЕ"},
            {"kind": "block", "type": "controls_repeat_ext", "label": "ПОВТОРИТЬ"},
            {"kind": "block", "type": "math_number", "label": "ЧИСЛО"},
            {"kind": "block", "type": "math_arithmetic", "label": "МАТЕМАТИКА"},
            {"kind": "block", "type": "variables_get", "label": "ПОЛУЧИТЬ ПЕРЕМЕННУЮ"},
            {"kind": "block", "type": "variables_set", "label": "УСТАНОВИТЬ ПЕРЕМЕННУЮ"}
        ]
    };

    const workspace = Blockly.inject('blocklyDiv', {
        toolbox: toolbox,
        toolboxPosition: 'start',
        trashcan: true,
        scrollbars: true,
        sounds: false
    });

    console.log("✅ Blockly успешно инициализирован с расширенным набором блоков.");
    return workspace;
}

// ✅ Экспорт функций
export { Blockly, pythonGenerator, initBlockly };
