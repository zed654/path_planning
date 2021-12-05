#include <iostream>
#include <string>
#include <vector>
#include <bitset>
#include <cmath>
#include <utility>  // For pair
#include <stdlib.h> // For srand
#include <time.h> // For seed of srand (use time)
#include <unistd.h> // For usleep

/////////////////////
/////////////////////
//// Debug Flag  ////
/////////////////////
/////////////////////
// #define __PRINT__          // Compare Selection, Crossover, Mutation
#define __PRINT2__          // Result with Chrom, Fit, (x1, x2), Total Fit, ProbDart, ProbCumDart
// #define __PRINT3__ // Result Only Gen (Best Fit, Worst Fit, Total Fit, nMutation)


///////////////////
///////////////////
//// Parameter ////
///////////////////
///////////////////
#define LEN_X1 18
#define LEN_X2 15
#define LEN LEN_X1 + LEN_X2
#define POP_SIZE 5
#define Pc 0.20 // Probability of Crossover // Assumption Best : 0.20
#define Pm 0.01 // Probability of Mutation  // Assumption Best : 0.01

class GA
{
private:
protected:
public:
    struct Pos
    {
        double x1;
        double x2;
    };
    struct Chromosome
    {
        unsigned long genes;
        double fitness;
        double prob_sel;
        double prob_cum_sel;
        Pos pos;
    };
    std::vector<Chromosome> chrom;
    std::vector<Chromosome> new_chrom;
    std::vector<int> dartboard;
    std::vector<std::pair<int, double> > expected_val;
    double total_fitness;
    std::vector<int> crossover_index;

    //FIXME: Delete
    int cut_point_tmp;
    int num_mutation;

    GA()
    {
        // srand((unsigned long)time(NULL)); // 얘 여기 넣어도 되는지 한 번 확인해봐야함. 내가 아는 이론상은 가능.
        srand((size_t)time(NULL)); // 얘 여기 넣어도 되는지 한 번 확인해봐야함. 내가 아는 이론상은 가능.
        InitVal();
    }
    void InitVal()
    {
        this->chrom.assign(POP_SIZE, {0, 0, 0, 0, {0, 0}}); // POP_SIZE에 의한 chromosome 사이즈 정의
        this->dartboard.assign(100, 0);                     // 숫자 100은 0~99% 를 나타냄.
        this->total_fitness = 0;
    }

    int GenRandInteger(int max_value_ = 10000000)
    {
        // Depended on <ctime> ... Have to use 'srand((size_t)time(NULL))'
        int range_max = max_value_;
        int range_min = 0;

        int tmp = (double)rand() / (RAND_MAX + 1) * (range_min - range_max) + range_min;
        return tmp;
    }
    void GenChromosomes()
    {
        // Random으로 초기 Chromosomes를 생성한다.
        for (int i = 0; i < POP_SIZE; i++)
        {
            unsigned long chrom_tmp = 0;
            for (int j = 0; j < LEN; j++)
            {
                chrom_tmp = chrom_tmp << 1;
                // chrom_tmp += GenRandBit(1);
                chrom_tmp += GenRandInteger(2);
            }
            this->chrom[i].genes = chrom_tmp;
        }

        // // FIXME: Fixed genes for debug
        // for(int i = 0; i < POP_SIZE; i++)
        // {
        //     this->chrom[i].genes = 0b110011101001100111010110100101010; //0b111111111111111111111111111111111;
        //     // this->chrom[i].genes = 0b111111111111111111111111111111111; //0b111111111111111111111111111111111;
        // }
    }
    void RenewChromosomes()
    {
        this->chrom = this->new_chrom;
    }

    void GetFitness()
    {
        total_fitness = 0;
        for (int i = 0; i < POP_SIZE; i++)
        {
            Pos x_tmp = BitDecoder(DetachChrom(this->chrom[i].genes)); // Binary로 합하여 표현된 x1-x2를 x1, x2로 분리
            this->chrom[i].pos = x_tmp;
            double fitness_tmp = GetFitness(x_tmp);
            this->chrom[i].fitness = fitness_tmp;
            total_fitness += fitness_tmp;
        }

        // Set Probability of Selection
        for (int i = 0; i < POP_SIZE; i++)
        {
            double fitness_tmp = this->chrom[i].fitness;
            this->chrom[i].prob_sel = fitness_tmp / total_fitness;

            // Calc cumulative Probability used on Dartboard
            if (i == 0)
                this->chrom[i].prob_cum_sel = fitness_tmp / total_fitness;
            else
                this->chrom[i].prob_cum_sel = this->chrom[i - 1].prob_cum_sel + fitness_tmp / total_fitness;
        }
        // 마지막 값이 0.9999 혹은 1.0000...1 이 되는걸 막기 위해 1을 넣어줌.
        this->chrom[POP_SIZE - 1].prob_cum_sel = 1;

#ifdef __PRINT__
        std::cout << "\t\t\t\tChromosomes\t\t\t\t\t\t\tFitness \t\t(x1, x2)\t\t\tTotal Fitness\tProbDartboard \tProbCumDartboard" << std::endl;
        for (int j = 0; j < POP_SIZE; j++)
        {
            std::cout << std::fixed;
            std::cout.precision(4);
            std::bitset<LEN> x(chrom[j].genes);
            // std::cout << "Gen  [" << i << "] : " << x << "\t\t" << ga.chrom[j].genes << "\t\t" << ga.chrom[j].fitness << "\t\t(" << ga.chrom[j].pos.x1 << ", " << ga.chrom[j].pos.x2 << ")\t\t" << ga.total_fitness << "\t\t" << ga.chrom[j].prob_sel << "\t\t" << ga.chrom[j].prob_cum_sel << std::endl;
            std::cout << "Input Genes : " << x << "\t\t" << this->chrom[j].fitness << "\t\t(" << this->chrom[j].pos.x1 << ", " << this->chrom[j].pos.x2 << ")\t\t" << this->total_fitness << "\t\t\t" << this->chrom[j].prob_sel << "\t\t" << this->chrom[j].prob_cum_sel << std::endl;
        }
        std::cout << std::endl;
#endif
    }

    void GetExpectedValue()
    {
        this->expected_val.clear();

        // 기대치 계산 및 저장
        for (int i = 0; i < POP_SIZE; i++)
        {
            double exp_tmp = this->chrom[i].prob_sel * POP_SIZE;
            this->expected_val.push_back({i, exp_tmp});
        }
    }
    void SelectionBasedOnExpect()
    {
        this->new_chrom.clear();

        // 기대치 계산 및 저장
        for (int i = 0; i < POP_SIZE; i++)
        {
            // 정수부가 1보다 크면 다음 세대로 이동
            double value = this->expected_val[i].second;
            double integer = (int)value;
            double fraction = value - (int)value;
            if (integer > 0)
            {
                this->new_chrom.push_back(this->chrom[i]);
            }
        }

        // 기대치의 소수부 정렬
        for (int i = 0; i < POP_SIZE - 1; i++)
        {
            std::pair<int, double> tmp = this->expected_val[i];
            for (int j = i + 1; j < POP_SIZE; j++)
            {
                std::pair<int, double> tmp2 = this->expected_val[j];
                if (tmp2.second - (int)tmp2.second > tmp.second - (int)tmp.second)
                {
                    this->expected_val[i] = tmp2;
                    this->expected_val[j] = tmp;
                    tmp = tmp2;
                }
            }
        }

        // 정렬된 기대치의 소수부가 큰 순서대로 다음 세대로 이동
        int integer_size = this->new_chrom.size();
        for (int i = 0; i < POP_SIZE - integer_size; i++)
        {
            this->new_chrom.push_back(this->chrom[this->expected_val[i].first]);
        }

#ifdef __PRINT__
        for (int i = 0; i < POP_SIZE; i++)
        {
            std::bitset<LEN> genes_tmp(new_chrom[i].genes);
            std::cout << "After Selection(Dartboard)[" << i << "] : " << genes_tmp << std::endl;
        }
        std::cout << std::endl;
#endif
    }
    void SelectionBasedOnExpectAndDartboard()
    {
        this->new_chrom.clear();

        // 기대치 계산 및 저장
        for (int i = 0; i < POP_SIZE; i++)
        {
            // 정수부가 1보다 크면 다음 세대로 이동
            double value = this->expected_val[i].second;
            double integer = (int)value;
            double fraction = value - (int)value;
            if (integer > 0)
            {
                this->new_chrom.push_back(this->chrom[i]);
            }
        }

        int integer_size = this->new_chrom.size();

        // 소수부 총 합 구하기.
        double total_frac_tmp = 0;
        for (int i = 0; i < POP_SIZE; i++)
        {
            double fraction_tmp = this->expected_val[i].second - (int)this->expected_val[i].second;
            total_frac_tmp += fraction_tmp;
        }

        // 소수부 기반 각각 유전자에 대한 누적 확률 구하기
        std::vector<std::pair<int, double> > accum_prob_based_on_frac;
        double cum_val = 0;
        for (int i = 0; i < POP_SIZE; i++)
        {
            double fraction_tmp = this->expected_val[i].second - (int)this->expected_val[i].second;
            cum_val += fraction_tmp / total_frac_tmp;
            // chrom의 index와 소수부 기반 확률 확률이 저장됨. (각각 first, second)
            accum_prob_based_on_frac.push_back({this->expected_val[i].first, cum_val}); // prev chrom의 index와 누적확률값 저장
        }
        accum_prob_based_on_frac[POP_SIZE - 1].second = 1;

        // Darboard 만들기
        int prev_index = 0;
        dartboard.clear();
        for (int i = 0; i < POP_SIZE; i++)
        {
            int index = std::ceil(accum_prob_based_on_frac[i].second * 100); // 10 -> 100 // ceil -> 반올림
            for (int j = prev_index; j < index; j++)
                dartboard[j] = i;
            prev_index = index;
        }

        // Dartboard에 Dart 던지기
        std::vector<int> chrom_index_tmp;
        for (int i = 0; i < POP_SIZE; i++)
        {
            double rand_num = (double)GenRandInteger(9900) / 10000.; // 소수점 4 자리까지 구하기. ( 0~0.9899 )
            int index_tmp = std::ceil(rand_num * 100);               // index 구하기. (0~99)
            chrom_index_tmp.push_back(dartboard[index_tmp]);   // Dartboard에 던지기. ( Index 범위는 0~99 )
        }

        // Renew Chromosome
        for (int i = 0; i < POP_SIZE-integer_size; i++)
        {
            new_chrom.push_back(this->chrom[chrom_index_tmp[i]]);
        }

#ifdef __PRINT__
        for (int i = 0; i < POP_SIZE; i++)
        {
            std::bitset<LEN> genes_tmp(new_chrom[i].genes);
            std::cout << "After Selection(Dartboard)[" << i << "] : " << genes_tmp << std::endl;
        }
        std::cout << std::endl;
#endif
    }

    void SetDartBoard()
    {
        int prev_index = 0;
        dartboard.clear();
        for (int i = 0; i < POP_SIZE; i++)
        {
            int index = std::ceil(this->chrom[i].prob_cum_sel * 100); // 10 -> 100 // ceil -> 반올림
            for (int j = prev_index; j < index; j++)
                dartboard[j] = i;
            prev_index = index;
        }

        ///////////////
        // For Debug //
        ///////////////
        // std::cout << dartboard.size() << std::endl;
        // for (int i = 0; i < dartboard.size(); i++)
        // {
        //     std::cout << "cum_index : " << dartboard[i] << "\t\t" << i << std::endl;
        // }
    }
    void Selection()
    {
        std::vector<int> chrom_index_tmp;
        for (int i = 0; i < POP_SIZE; i++)
        {
            // double rand_num = (double)GenRandBit(9899) / 10000.;   // 소수점 4 자리까지 구하기. ( 0~0.9899 )
            double rand_num = (double)GenRandInteger(9900) / 10000.; // 소수점 4 자리까지 구하기. ( 0~0.9899 )
            int index_tmp = std::ceil(rand_num * 100);               // index 구하기. (0~99)
            chrom_index_tmp.push_back(this->dartboard[index_tmp]);   // Dartboard에 던지기. ( Index 범위는 0~99 )
        }

        new_chrom.clear();
        for (int i = 0; i < POP_SIZE; i++)
        {
            new_chrom.push_back(this->chrom[chrom_index_tmp[i]]);
        }
#ifdef __PRINT__
        for (int i = 0; i < POP_SIZE; i++)
        {
            std::bitset<LEN> genes_tmp(new_chrom[i].genes);
            std::cout << "After Selection(Dartboard)[" << i << "] : " << genes_tmp << std::endl;
        }
        std::cout << std::endl;
#endif
    }

    void Crossover()
    {
        std::vector<double> prob_crossover;
        
        crossover_index.clear();
        while (crossover_index.size() < 2) // Crossover할 Chromsome을 짝수개로 맞춰주기 위한 조건
        {
            // Pc가 0인 경우 무한 루프에 빠지는 문제 방지
            if (Pc == 0)
                break;

            prob_crossover.clear();
            crossover_index.clear();
            // 각각의 Chromosome에 대한 Crossover 확률 구하기
            for (int i = 0; i < POP_SIZE; i++)
            {
                double rand_num = (double)GenRandInteger(10001) / 10000.;
                // std::cout << rand_num << std::endl;
                prob_crossover.push_back(rand_num);
            }

            // Crossover 확률 Pc보다 낮은 Chromosome 찾아서 Index 저장
            for (int i = 0; i < POP_SIZE; i++)
                if (Pc > prob_crossover[i])
                    crossover_index.push_back(i);
        }

        // crossover를 진행할 chrom이 홀수이면, 하나를 제거해서 짝수개를 맞춤.
        if ((crossover_index.size() % 2) != 0)
            crossover_index.erase(crossover_index.begin());


        // 18 bit (LEN_X1)
        // 15 bit (LEN_X2)
        // 33 bit (LEN) // unsigned long (64 bit)
        // 우측단 복사 1, 우측단 전부 0으로 (cut_point만큼 >> 하고 <<)
        // 우측단 복사 2, 우측단 전부 0으로 (cut_point만틈 >> 하고 <<)
        // 복사 2를 1에 or연산
        // 복수 1을 2에 or연산
        for (int i = 0; i < crossover_index.size(); i = i + 2)
        {
            // Rand로 cut-point 설정
            int cut_point = SetCutPoints(LEN);
            // int cut_point = SetCutPoints(LEN - 1);

            //FIXME: Delete
            //FIXME: Delete
            this->cut_point_tmp = cut_point;

            unsigned long chrom_tmp = this->new_chrom[crossover_index[i]].genes;      //0b100111111111111111111111111111010;
            unsigned long chrom_tmp2 = this->new_chrom[crossover_index[i + 1]].genes; //0b111111111111111111111111111110101;
            unsigned long new_chrom_tmp;
            unsigned long new_chrom_tmp2;

            std::bitset<LEN> x1_tmp(chrom_tmp);
            std::bitset<LEN> x2_tmp(chrom_tmp2);

            // Cut Point의 사이즈에 맞게 모두 1로 선언된 reference chromosome을 생성함. 얘로 and연산 할꺼임.
            unsigned long tmp = 1;
            for (int i = 0; i < cut_point - 1; i++)
                tmp = (tmp << 1) + 1;

            // Pair Chromosome에 대해 Cut Point부분을 자른 후, 데이터를 저장해 둠
            unsigned long x1_cut_section;
            x1_cut_section = chrom_tmp & tmp;
            unsigned long x2_cut_section;
            x2_cut_section = chrom_tmp2 & tmp;
            x1_tmp = x1_cut_section;
            x2_tmp = x2_cut_section;
            // std::cout << "Cut   : " << x1_tmp << "\t\t" << x2_tmp << std::endl;

            // Pair Chromosome의 잘린 부분에 대해 0으로 값을 치환
            chrom_tmp = (chrom_tmp >> cut_point) << cut_point;
            chrom_tmp2 = (chrom_tmp2 >> cut_point) << cut_point;
            x1_tmp = chrom_tmp;
            x2_tmp = chrom_tmp2;
            // std::cout << "L_Zero: " << x1_tmp << "\t\t" << x2_tmp << std::endl;

            // Pair Chromosome의 Cut Point에 대한 Crossover 접합
            new_chrom_tmp = chrom_tmp | x2_cut_section;
            new_chrom_tmp2 = chrom_tmp2 | x1_cut_section;
            x1_tmp = new_chrom_tmp;
            x2_tmp = new_chrom_tmp2;
            // std::cout << "2 Gen : " << x1_tmp << "\t\t" << x2_tmp << "\t\t"
            //           << "Before Mutation, After Crossover" << std::endl
            //           << std::endl;

            // reproduction에서 갱신된 chromosome을 crossover된 chromosome으로 갱신해줌.
            this->new_chrom[crossover_index[i]].genes = new_chrom_tmp;
            this->new_chrom[crossover_index[i + 1]].genes = new_chrom_tmp2;

#ifdef __PRINT__
            std::cout << "CutPoint : " << cut_point_tmp << "\t\t CrossId : " << crossover_index[i] << "\t\t" << crossover_index[i + 1] << std::endl;
#endif
        }
#ifdef __PRINT__
        for (int i = 0; i < POP_SIZE; i++)
        {
            std::bitset<LEN> genes_tmp(new_chrom[i].genes);
            std::cout << "After Crossover[" << i << "] : " << genes_tmp << std::endl;
        }
        std::cout << std::endl;
#endif
    }

    void Mutation()
    {
        unsigned long chrom_tmp = 0;
        unsigned long new_chrom_tmp = 0;
        num_mutation = 0;
        for (int i = 0; i < POP_SIZE; i++)
        {
            chrom_tmp = this->new_chrom[i].genes;
            new_chrom_tmp = 0;
            for (int j = 0; j < LEN; j++)
            {
                double rand_tmp = GenRandInteger(10000000) / 10000000.;
                unsigned long tmp = (chrom_tmp >> j) & 0b1; // tmp 는 0 혹은 1의 변수. bool 쓰면 안됨. 이유는 뒤에서 shift하기 때문에 메모리를 할당해놔야함.

                if (rand_tmp < Pm)
                {
                    tmp = 1 - tmp; // Flip the value
                    num_mutation++;
                    // std::cout.precision(10);
                    // std::cout << rand_tmp << "\t\t" << std::endl;
                }

                new_chrom_tmp = new_chrom_tmp + (tmp << j);
            }

            new_chrom[i].genes = new_chrom_tmp;

            // std::cout << std::endl;
            // std::bitset<LEN> print(new_chrom_tmp);
            // std::cout << "renew : " << print << std::endl;
        }
#ifdef __PRINT__
        for (int i = 0; i < POP_SIZE; i++)
        {
            std::bitset<LEN> genes_tmp(new_chrom[i].genes);
            std::cout << "After Mutation[" << i << "]  : " << genes_tmp << std::endl;
        }
        std::cout << std::endl;
#endif
    }

    int SetCutPoints(int max_value_)
    {
        int cut_point_tmp = 0;
        while (cut_point_tmp == 0)
        {
            cut_point_tmp = GenRandInteger(max_value_);
        }

        return cut_point_tmp;
    }

    Pos BitDecoder(std::vector<unsigned long> encoded_bit_)
    {
        Pos x_tmp = {
            -3.0 + (double)encoded_bit_[0] * (12.1 - (-3.0)) / (std::pow(2, 18) - 1),
            4.1 + (double)encoded_bit_[1] * (5.8 - 4.1) / (std::pow(2, 15) - 1)};
        return x_tmp;
    }

    std::vector<unsigned long> DetachChrom(unsigned long chrom_)
    {
        unsigned long ref = chrom_; // 33 bits

        // long 64bit
        std::vector<unsigned long> x_tmp;
        x_tmp.push_back(ref >> LEN_X2);

        int bit_shift_size_tmp = sizeof(unsigned long) * 8 - LEN_X2;
        unsigned long tmp = ref << (bit_shift_size_tmp);
        x_tmp.push_back(tmp >> bit_shift_size_tmp);

        return x_tmp;
    }

    double GetFitness(Pos x_)
    {
        double fitness_tmp = 21.5 + x_.x1 * std::sin(4 * M_PI * x_.x1) + x_.x2 * std::sin(20 * M_PI * x_.x2);
        return fitness_tmp;
    }
};

int main(int argc, char **argv)
{
    GA ga;

    for (int i = 0; i < 10; i++)
    {
        ////////////
        // Step 1 //
        ////////////
        if (i == 0)
            ga.GenChromosomes(); // Population
        else
            ga.RenewChromosomes(); // 다음 세대의 chromosomes를 가져온다.

        //////////////////////////////////////////////////////////////////////////
        // Step 2. Reproduction Methods 1(Stochastic Sampling with replacement) //
        //////////////////////////////////////////////////////////////////////////
        // ga.GetFitness();
        // ga.SetDartBoard();
        // ga.Selection();

        //////////////////////////////////////////////////////////////////////////
        //////// Step 2. Reproduction Methods 2(Deterministic Sampling) //////////
        //////////////////////////////////////////////////////////////////////////
        // ga.GetFitness();
        // ga.GetExpectedValue();
        // ga.SelectionBasedOnExpect();

        
        /////////////////////////////////////////////////////////////////////////////////////////
        //// Step 2. Reproduction Methods 3(Remainder Stochastic Sampling with replacement) /////
        /////////////////////////////////////////////////////////////////////////////////////////
        ga.GetFitness();
        ga.GetExpectedValue();
        ga.SelectionBasedOnExpectAndDartboard();

        ///////////////////////
        // Step 3. Crossover //
        ///////////////////////
        ga.Crossover();

        //////////////////////
        // Step 4. Mutation //
        //////////////////////
        ga.Mutation();

#ifdef __PRINT2__
        double comp_tmp = 0;
        double comp_tmp2 = 9999;
        for (int j = 0; j < ga.chrom.size(); j++)
        {
            if (ga.chrom[j].fitness > comp_tmp)
                comp_tmp = ga.chrom[j].fitness;

            if (ga.chrom[j].fitness < comp_tmp2)
                comp_tmp2 = ga.chrom[j].fitness;
        }
        std::cout << "\t\t\t\tChromosomes\t\t\t\t\t\t\tFitness \t\t(x1, x2)\t\t\tTotal Fitness\tProbDartboard \tProbCumDartboard" << std::endl;
        for (int j = 0; j < POP_SIZE; j++)
        {
            std::bitset<LEN> x(ga.chrom[j].genes);
            std::cout << std::fixed;
            std::cout.precision(4);
            // std::cout << "Gen  [" << i << "] : " << x << "\t\t" << ga.chrom[j].genes << "\t\t" << ga.chrom[j].fitness << "\t\t(" << ga.chrom[j].pos.x1 << ", " << ga.chrom[j].pos.x2 << ")\t\t" << ga.total_fitness << "\t\t" << ga.chrom[j].prob_sel << "\t\t" << ga.chrom[j].prob_cum_sel << std::endl;
            std::cout << "Input Genes : " << x << "\t\t" << ga.chrom[j].fitness << "\t\t(" << ga.chrom[j].pos.x1 << ", " << ga.chrom[j].pos.x2 << ")\t\t" << ga.total_fitness << "\t\t\t" << ga.chrom[j].prob_sel << "\t\t" << ga.chrom[j].prob_cum_sel << std::endl;
        }
        std::cout << std::endl;
        std::cout << "Gen : " << i << "\t\tBest Fitness : " << comp_tmp << "\t\tWorst Fitness : " << comp_tmp2 << "\t\tTotal Fitness : " << ga.total_fitness << "\t\tNum of Mutation : " << ga.num_mutation << std::endl;
#endif

#ifdef __PRINT3__
        double comp_tmp = 0;
        double comp_tmp2 = 9999;
        for (int j = 0; j < ga.chrom.size(); j++)
        {
            if (ga.chrom[j].fitness > comp_tmp)
                comp_tmp = ga.chrom[j].fitness;

            if (ga.chrom[j].fitness < comp_tmp2)
                comp_tmp2 = ga.chrom[j].fitness;
        }
        std::cout << "Gen : " << i << "\t\tBest Fitness : " << comp_tmp << "\t\tWorst Fitness : " << comp_tmp2 << "\t\tTotal Fitness : " << ga.total_fitness << "\t\tNum of Mutation : " << ga.num_mutation << "\t\tNum of Crossover : " << ga.crossover_index.size() << std::endl;
#endif

#ifdef __PRINT__
        double comp_tmp = 0;
        double comp_tmp2 = 9999;
        for (int j = 0; j < ga.chrom.size(); j++)
        {
            if (ga.chrom[j].fitness > comp_tmp)
                comp_tmp = ga.chrom[j].fitness;

            if (ga.chrom[j].fitness < comp_tmp2)
                comp_tmp2 = ga.chrom[j].fitness;
        }
        std::cout << "Gen : " << i << "\t\tBest Fitness : " << comp_tmp << "\t\tWorst Fitness : " << comp_tmp2 << "\t\tCut Point : " << ga.cut_point_tmp << "\t\tNum of Mutation : " << ga.num_mutation << std::endl;
        std::cout << "------------------------------------------------------------------------------------------------" << std::endl;
        std::cout << std::endl
                  << std::endl;
#endif
    }
}
