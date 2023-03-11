# protoc --plugin=protoc-gen-eams=protoc-gen-eams -I LOCATION/PROTO/FILES --eams_out=GENERATED/SRC/DIR PROTO_MESSAGE_FILE.proto
echo "SCRIPT NEEDS TO BE CALLED FROM WITHIN THE ./firmware/proto FOLDER. IS THIS THE CASE?"

# from https://stackoverflow.com/a/18546416
read -p "Continue? (Y/N): " confirm && [[ $confirm == [yY] || $confirm == [yY][eE][sS] ]] || exit 1

echo "Changing to EmbeddedProto directory..."
cd ../../
cd EmbeddedProto

echo "Generating proto files.."
protoc --plugin=protoc-gen-eams=protoc-gen-eams -I ../firmware/proto --eams_out=../firmware/Platformio/include ../firmware/proto/uart_messages.proto
protoc --plugin=protoc-gen-eams=protoc-gen-eams -I ../firmware/proto --python_out=../firmware/desktop ../firmware/proto/uart_messages.proto

echo "Checking if EmbeddedProto/src exists..."
DIRECTORY="../EmbeddedProto/src"
if [ -d "$DIRECTORY" ]; then
    echo "$DIRECTORY does exists"
    echo "Copying EmbeddedProto source code to Platformio..."
    cp ../EmbeddedProto/src/* ../firmware/Platformio/lib/embeddedproto
else
    echo "EmbeddedProto source code missing, exiting..."
    exit
fi

echo "j'ai fini!"
