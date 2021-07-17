echo "---started compilation---"
pio run --environment core2foraws
echo "---completed compilation---"
echo "---started upload---"
pio run --environment core2foraws --target upload
echo "---completed upload---"
echo "---started monitor---"
pio run --environment core2foraws --target monitor
echo "---completed monitor---"