import csv

for to_ind in (5, 10, 15, 20):
    for seed_i in range (1, 51):
        with open("2021-02-10_robots#20_timeout_const#{}_augmmented_knowledge#true/seed#{}_results.csv".format(to_ind,seed_i)) as csv_file:
            print(to_ind, "seed#{}_results.csv".format(seed_i))
            csv_reader = csv.reader(csv_file, delimiter=';')
            task_count = 0
            for row in csv_reader:
                if task_count == 0:
                    print(f'Column names are:\n\t{", ".join(row)}')
                    task_count += 1
                else:
                    print(f'\t{row[0]}\t{row[1]}\t{row[2]}\t{row[3]}\t{row[4]}\t{row[5]}')
                    task_count += 1
            print(f'Completed {task_count-1} tasks.')
        print('-------------------------------')
