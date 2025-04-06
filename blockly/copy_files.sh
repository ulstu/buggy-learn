docker cp ./blockly/send_cmd_server.py ulstub-devel:/ulstu/server
docker exec -it ulstub-devel bash -c 'rm -r /ulstu/server/static/build'
docker cp -a ./blockly/static/build ulstub-devel:/ulstu/server/static/
docker cp -a ./projects/Thymio2/ ulstub-devel:/ulstu/repositories/