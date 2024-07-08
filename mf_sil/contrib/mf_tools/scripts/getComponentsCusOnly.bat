ECHO ON
python getComponents.py --cus_only
python copyInterfaceHeaders.py --cus_only
python fetch.py -v cus_only