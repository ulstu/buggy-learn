import { Blockly, pythonGenerator, initBlockly } from './blockly.js';

document.addEventListener("DOMContentLoaded", function () {
    const workspace = initBlockly();

    console.log("Инициализация Blockly...");
    console.log("Blockly.Xml.textToDom availability:", typeof Blockly.Xml.textToDom); // Debug log

    // Extract world_path from URL
    const urlParams = new URLSearchParams(window.location.search);
    let worldPath = urlParams.get('world_path');
    if (worldPath == null)
        worldPath = "Thymio2"
    console.log('worldPath: ' + worldPath)

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

    const saveButton = document.getElementById("saveButton");
    const loadButton = document.getElementById("loadButton");
    const modal = document.getElementById("modal");
    const modalOverlay = document.getElementById("modalOverlay");
    const programNameInput = document.getElementById("programName");
    const saveProgramConfirm = document.getElementById("saveProgramConfirm");
    const cancelSave = document.getElementById("cancelSave");

    // Show modal for saving program
    saveButton.addEventListener("click", () => {
        modal.style.display = "block";
        modalOverlay.style.display = "block";
    });

    // Save program to server
    saveProgramConfirm.addEventListener("click", () => {
        const programName = programNameInput.value.trim();
        if (!programName) {
            alert("Имя программы не может быть пустым.");
            return;
        }

        const xml = Blockly.Xml.workspaceToDom(workspace);
        const xmlText = Blockly.Xml.domToText(xml);

        fetch('/api/save', {
            method: 'POST',
            headers: { 'Content-Type': 'application/json' },
            body: JSON.stringify({ name: programName, xml: xmlText, world_path: worldPath })
        })
        .then(response => {
            console.log("Request to /api/save:", response);  // Debug log
            return response.json();
        })
        .then(data => {
            if (data.status === "ok") {
                alert("Программа успешно сохранена.");
            } else {
                alert("Ошибка при сохранении программы.");
            }
        })
        .catch(error => console.error("Ошибка при сохранении программы:", error));

        modal.style.display = "none";
        modalOverlay.style.display = "none";
    });

    // Cancel saving program
    cancelSave.addEventListener("click", () => {
        modal.style.display = "none";
        modalOverlay.style.display = "none";
    });

    // Load program from server
    loadButton.addEventListener("click", () => {
        fetch('/api/list', {
            method: 'POST',
            headers: { 'Content-Type': 'application/json' },
            body: JSON.stringify({ world_path: worldPath })
        })
        .then(response => response.json())
        .then(data => {
            const programName = prompt("Выберите программу:\n" + data.programs.join("\n"));
            if (programName && data.programs.includes(programName)) {
                fetch('/api/load', {
                    method: 'POST',
                    headers: { 'Content-Type': 'application/json' },
                    body: JSON.stringify({ name: encodeURIComponent(programName), world_path: worldPath })
                })
                .then(response => response.json())
                .then(data => {
                    try {
                        if (!data.xml || typeof data.xml !== 'string') {
                            throw new Error("Invalid XML data received from the server.");
                        }
                        console.log("Received XML:", data.xml);
                        
                        // Safer workspace clearing logic
                        console.log("Clearing workspace...");
                        
                        try {
                            // Temporarily disable events to prevent errors during workspace clearing
                            Blockly.Events.disable();
                            
                            // Clear the workspace in one go
                            workspace.clear();
                            
                            // Reset undo/redo history
                            if (workspace.undoStack_) workspace.undoStack_.length = 0;
                            if (workspace.redoStack_) workspace.redoStack_.length = 0;
                            
                            // Clear all variables
                            if (workspace.variableMap_) workspace.variableMap_.clear();
                            
                            console.log("Workspace cleared successfully");
                        } catch (clearError) {
                            console.error("Error during workspace clearing:", clearError);
                        } finally {
                            // Always re-enable events after clearing
                            Blockly.Events.enable();
                        }
                        
                        console.log("Now loading program...");
                        
                        try {
                            const parser = new DOMParser();
                            const xmlDoc = parser.parseFromString(data.xml, 'text/xml');
                            
                            // Create blocks with events disabled to prevent errors
                            Blockly.Events.disable();
                            
                            // Create a map to store blocks by ID
                            const blockMap = new Map();
                            
                            // First pass: Create all blocks
                            xmlDoc.querySelectorAll('block').forEach(blockElem => {
                                // Process only top-level blocks
                                if (!blockElem.parentElement || 
                                    (blockElem.parentElement.tagName !== 'VALUE' && 
                                     blockElem.parentElement.tagName !== 'NEXT')) {
                                    
                                    const type = blockElem.getAttribute('type');
                                    const id = blockElem.getAttribute('id');
                                    const x = parseInt(blockElem.getAttribute('x') || '0', 10);
                                    const y = parseInt(blockElem.getAttribute('y') || '0', 10);
                                    
                                    if (type) {
                                        const block = workspace.newBlock(type, id);
                                        block.initSvg();
                                        block.moveTo({x: x, y: y});
                                        blockMap.set(id, block);
                                        
                                        processBlockContents(block, blockElem, blockMap);
                                    }
                                }
                            });
                            
                            // Re-enable events after all blocks are created and connected
                            Blockly.Events.enable();
                            
                            // Render all blocks after connections are made
                            blockMap.forEach(block => {
                                block.render();
                            });
                            
                            alert("Программа успешно загружена.");
                        } catch (error) {
                            // Re-enable events in case of error
                            Blockly.Events.enable();
                            console.error("Ошибка при импорте XML напрямую:", error);
                            alert("Ошибка при загрузке программы: " + error.message);
                        }
                    } catch (error) {
                        console.error("Ошибка при обработке XML:", error);
                        alert("Ошибка при обработке XML программы.");
                    }
                })
                .catch(error => console.error("Ошибка при загрузке программы:", error));
            }
        })
        .catch(error => console.error("Ошибка при получении списка программ:", error));
    });

    // Helper function to process block contents recursively
    function processBlockContents(block, blockElem, blockMap) {
        // Process field values
        blockElem.querySelectorAll(':scope > field').forEach(fieldElem => {
            const name = fieldElem.getAttribute('name');
            if (name && block.getField(name)) {
                block.getField(name).setValue(fieldElem.textContent);
            }
        });
        
        // Process value inputs (child blocks)
        blockElem.querySelectorAll(':scope > value').forEach(valueElem => {
            const name = valueElem.getAttribute('name');
            const childBlockElem = valueElem.querySelector(':scope > block');
            
            if (childBlockElem && name) {
                const childType = childBlockElem.getAttribute('type');
                const childId = childBlockElem.getAttribute('id');
                
                if (childType && childId) {
                    const childBlock = workspace.newBlock(childType, childId);
                    childBlock.initSvg();
                    blockMap.set(childId, childBlock);
                    
                    // Process child block recursively
                    processBlockContents(childBlock, childBlockElem, blockMap);
                    
                    // Connect child block to parent
                    if (block.getInput(name) && block.getInput(name).connection && 
                        childBlock.outputConnection) {
                        childBlock.outputConnection.connect(block.getInput(name).connection);
                    }
                }
            }
        });
        
        // Process next blocks (sequential blocks)
        const nextElem = blockElem.querySelector(':scope > next');
        if (nextElem) {
            const nextBlockElem = nextElem.querySelector(':scope > block');
            if (nextBlockElem) {
                const nextType = nextBlockElem.getAttribute('type');
                const nextId = nextBlockElem.getAttribute('id');
                
                if (nextType && nextId) {
                    const nextBlock = workspace.newBlock(nextType, nextId);
                    nextBlock.initSvg();
                    blockMap.set(nextId, nextBlock);
                    
                    // Process next block recursively
                    processBlockContents(nextBlock, nextBlockElem, blockMap);
                    
                    // Connect next block to current
                    if (block.nextConnection && nextBlock.previousConnection) {
                        nextBlock.previousConnection.connect(block.nextConnection);
                    }
                }
            }
        }
    }
});

