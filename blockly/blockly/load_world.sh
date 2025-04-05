curl -X POST http://localhost:1999/graphql -H "Content-Type: application/json" -d '{
    "query": "mutation { loadWorld(url: \"webots://webots://home/hiber/repositories/blockly/worlds/scene.wbt\") { success message } }"
}'

