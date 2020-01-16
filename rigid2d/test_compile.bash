rm test_output.txt
./rigid2d_test < test1_input.txt >> test_output.txt
cmp -s test_output.txt test1_answer.txt && echo "Success"
cmp -s test_output.txt test1_answer.txt || echo "Failure"