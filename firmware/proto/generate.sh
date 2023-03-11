# protoc --plugin=protoc-gen-eams=protoc-gen-eams -I LOCATION/PROTO/FILES --eams_out=GENERATED/SRC/DIR PROTO_MESSAGE_FILE.proto
# need to run this script with proto folder as cwd
cd ../../
cd EmbeddedProto
pwd
protoc --plugin=protoc-gen-eams=protoc-gen-eams -I ../firmware/proto --eams_out=../firmware/proto/gen ../firmware/proto/uart_messages.proto
protoc --plugin=protoc-gen-eams=protoc-gen-eams -I ../firmware/proto --python_out=../firmware/proto/gen ../firmware/proto/uart_messages.proto
