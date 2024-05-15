##Builds the app and copy it to the $HOME/.apps folder from where it will be launched at boot
cmake ..
make
cp rfm69app $HOME/.apps/rfm69app
