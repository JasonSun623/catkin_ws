#ifndef GALGO_CPP
#define GALGO_CPP
#include "Galgo.hpp"
///Chromosome
namespace galgo {

/*-------------------------------------------------------------------------------------------------*/

// constructor
template <typename T>
Chromosome<T>::Chromosome(const GeneticAlgorithm<T>& ga)
{
   param.resize(ga.nbparam);
   ptr = &ga;
   chrsize = ga.nbbit;
   numgen = ga.nogen;
}

/*-------------------------------------------------------------------------------------------------*/

// copy constructor
template <typename T>
Chromosome<T>::Chromosome(const Chromosome<T>& rhs)
{
   param = rhs.param;
   result = rhs.result;
   chr = rhs.chr;
   ptr = rhs.ptr;
   // re-initializing fitness to its original value
   fitness = rhs.total;
   total = rhs.total;
   chrsize = rhs.chrsize;
   numgen = rhs.numgen;
}

/*-------------------------------------------------------------------------------------------------*/

// create new chromosome
template <typename T>
inline void Chromosome<T>::create()
{
   chr.clear();

   for (const auto& x : ptr->param) {
      // encoding parameter random value
      std::string str = x->encode();
      chr.append(str);
   }
}

/*-------------------------------------------------------------------------------------------------*/

// initialize chromosome (from known parameter values)
template <typename T>
inline void Chromosome<T>::initialize()
{
   chr.clear();

   int i(0);
   for (const auto& x : ptr->param) {
      // encoding parameter initial value
      std::string str = x->encode(ptr->initialSet[i++]);
      chr.append(str);
   }
}

/*-------------------------------------------------------------------------------------------------*/

// evaluate chromosome fitness
template <typename T>
inline void Chromosome<T>::evaluate()
{
   int i(0);
   for (const auto& x : ptr->param) {
      // decoding chromosome: converting chromosome string into a real value
      param[i] = x->decode(chr.substr(ptr->idx[i], x->size()));
      i++;
   }
   // computing objective result(s)
   result = ptr->Objective(param);
   // computing sum of all results (in case there is not only one objective functions)
   total = std::accumulate(result.begin(), result.end(), 0.0);
   // initializing fitness to this total
   fitness = total;
}

/*-------------------------------------------------------------------------------------------------*/

// reset chromosome
template <typename T>
inline void Chromosome<T>::reset()
{
   chr.clear();
   result = 0.0;
   total = 0.0;
   fitness = 0.0;
}

/*-------------------------------------------------------------------------------------------------*/

// set or replace kth gene by a new one
template <typename T>
inline void Chromosome<T>::setGene(int k)
{
   #ifndef NDEBUG
   if (k < 0 || k >= ptr->nbparam) {
      throw std::invalid_argument("Error: in galgo::Chromosome<T>::setGene(int), argument cannot be outside interval [0,nbparam-1], please amend.");
   }
   #endif

   // generating a new gene
   std::string s = ptr->param[k]->encode();
   // adding or replacing gene in chromosome
   chr.replace(ptr->idx[k], s.size(), s, 0, s.size());
}

/*-------------------------------------------------------------------------------------------------*/

// initialize or replace kth gene by a know value
template <typename T>
inline void Chromosome<T>::initGene(int k, T x)
{
   #ifndef NDEBUG
   if (k < 0 || k >= ptr->nbparam) {
      throw std::invalid_argument("Error: in galgo::Chromosome<T>::initGene(int), first argument cannot be outside interval [0,nbparam-1], please amend.");
   }
   #endif

   // encoding gene
   std::string s = ptr->param[k]->encode(x);
   // adding or replacing gene in chromosome
   chr.replace(ptr->idx[k], s.size(), s, 0, s.size());
}

/*-------------------------------------------------------------------------------------------------*/

// add chromosome bit to chromosome (when constructing a new one)
template <typename T>
inline void Chromosome<T>::addBit(char bit)
{
   chr.push_back(bit);

   #ifndef NDEBUG
   if (chr.size() > chrsize) {
      throw std::out_of_range("Error: in galgo::Chromosome<T>::setBit(char), exceeding chromosome size.");
   }
   #endif
}

/*-------------------------------------------------------------------------------------------------*/

// initialize or replace an existing chromosome bit
template <typename T>
inline void Chromosome<T>::setBit(char bit, int pos)
{
   #ifndef NDEBUG
   if (pos >= chrsize) {
      throw std::out_of_range("Error: in galgo::Chromosome<T>::replaceBit(char, int), second argument cannot be equal or greater than chromosome size.");
   }
   #endif

   std::stringstream ss;
   std::string str;
   ss << bit;
   ss >> str;
   chr.replace(pos, 1, str);
   std::cout << chr << "\n";
}

/*-------------------------------------------------------------------------------------------------*/

// flip an existing chromosome bit
template <typename T>
inline void Chromosome<T>::flipBit(int pos)
{
   #ifndef NDEBUG
   if (pos >= chrsize) {
      throw std::out_of_range("Error: in galgo::Chromosome<T>::flipBit(int), argument cannot be equal or greater than chromosome size.");
   }
   #endif

   if (chr[pos] == '0') {
      chr.replace(pos, 1, "1");
   } else {
      chr.replace(pos, 1, "0");
   }
}

/*-------------------------------------------------------------------------------------------------*/

// get a chromosome bit
template <typename T>
inline char Chromosome<T>::getBit(int pos) const
{
   #ifndef NDEBUG
   if (pos >= chrsize) {
      throw std::out_of_range("Error: in galgo::Chromosome<T>::getBit(int), argument cannot be equal or greater than chromosome size.");
   }
   #endif

   return chr[pos];
}

/*-------------------------------------------------------------------------------------------------*/

// initialize or replace a portion of bits with a portion of another chromosome (from position start to position end included)
template <typename T>
inline void Chromosome<T>::setPortion(const Chromosome<T>& x, int start, int end)
{
   #ifndef NDEBUG
   if (start > chrsize) {
      throw std::out_of_range("Error: in galgo::Chromosome<T>::setPortion(const Chromosome<T>&, int, int), second argument cannot be greater than chromosome size.");
   }
   #endif

   chr.replace(start, end - start + 1, x.chr, start, end - start + 1);
}

/*-------------------------------------------------------------------------------------------------*/

// initialize or replace a portion of bits with a portion of another chromosome (from position start to the end of he chromosome)
template <typename T>
inline void Chromosome<T>::setPortion(const Chromosome<T>& x, int start)
{
   #ifndef NDEBUG
   if (start > chrsize) {
      throw std::out_of_range("Error: in galgo::Chromosome<T>::setPortion(const Chromosome<T>&, int), second argument cannot be greater than chromosome size.");
   }
   #endif

   chr.replace(start, chrsize, x.chr, start, x.chrsize);
}

/*-------------------------------------------------------------------------------------------------*/

// get parameter value(s) from chromosome
template <typename T>
inline const std::vector<T>& Chromosome<T>::getParam() const
{
   return param;
}

/*-------------------------------------------------------------------------------------------------*/

// get objective function result
template <typename T>
inline const std::vector<T>& Chromosome<T>::getResult() const
{
   return result;
}

/*-------------------------------------------------------------------------------------------------*/

// get the total sum of all objective function(s) result
template <typename T>
inline T Chromosome<T>::getTotal() const
{
   return total;
}

/*-------------------------------------------------------------------------------------------------*/

// get constraint value(s) for this chromosome
template <typename T>
inline const std::vector<T> Chromosome<T>::getConstraint() const
{
   return ptr->Constraint(param);
}

/*-------------------------------------------------------------------------------------------------*/

// return chromosome size in number of bits
template <typename T>
inline int Chromosome<T>::size() const
{
   return chrsize;
}

/*-------------------------------------------------------------------------------------------------*/

// return mutation rate
template <typename T>
inline T Chromosome<T>::mutrate() const
{
   return ptr->mutrate;
}

/*-------------------------------------------------------------------------------------------------*/

// return number of genes in chromosome
template <typename T>
inline int Chromosome<T>::nbgene() const
{
   return ptr->nbparam;
}

/*-------------------------------------------------------------------------------------------------*/

// return numero of generation this chromosome belongs to
template <typename T>
inline int Chromosome<T>::nogen() const
{
   return numgen;
}

/*-------------------------------------------------------------------------------------------------*/

// return lower bound(s)
template <typename T>
inline const std::vector<T>& Chromosome<T>::lowerBound() const
{
   return ptr->lowerBound;
}

/*-------------------------------------------------------------------------------------------------*/

// return upper bound(s)
template <typename T>
inline const std::vector<T>& Chromosome<T>::upperBound() const
{
   return ptr->upperBound;
}


/// common

// In this header, the user can define his own selection, cross-over, mutation and adaptation to
// constraint(s) methods by respecting the function declaration template

//=================================================================================================

// SELECTION METHODS

/*-------------------------------------------------------------------------------------------------*/

// proportional roulette wheel selection
template <typename T>
void RWS(galgo::Population<T>& x)
{
   // adjusting all fitness to positive values
   x.adjustFitness();
   // computing fitness sum
   T fitsum = x.getSumFitness();

   // selecting mating population
   for (int i = 0, end = x.matsize(); i < end; ++i) {
      // generating a random fitness sum in [0,fitsum)
      T fsum = galgo::uniform<T>(0.0, fitsum);

      int j = 0;
      while (fsum >= 0.0) {
         #ifndef NDEBUG
         if (j == x.popsize()) {
            throw std::invalid_argument("Error: in RWS(galgo::Population<T>&) index j cannot be equal to population size.");
         }
         #endif
         fsum -= x(j)->fitness;
         j++;
      }
      // selecting element
      x.select(j - 1);
   }
}

/*-------------------------------------------------------------------------------------------------*/

// stochastic universal sampling selection
template <typename T>
void SUS(galgo::Population<T>& x)
{
   // adjusting all fitness to positive values
   x.adjustFitness();
   // computing fitness sum
   T fitsum = x.getSumFitness();

   int matsize = x.matsize();
   // computing interval size
   T dist = fitsum / matsize;
   // initializing pointer
   T ptr = galgo::uniform<T>(0.0, dist);

   // selecting mating population
   for (int i = 0; i < matsize; ++i) {

      int j = 0;
      T fsum = 0;

      while (fsum <= ptr) {
         #ifndef NDEBUG
         if (j == x.popsize()) {
            throw std::invalid_argument("Error: in SUS(galgo::Population<T>&) index j cannot be equal to population size.");
         }
         #endif
         fsum += x(j)->fitness;
         j++;
      }
      // selecting element
      x.select(j - 1);

      // incrementing pointer
      ptr += dist;
   }
}

/*-------------------------------------------------------------------------------------------------*/

// classic linear rank-based selection
template <typename T>
void RNK(galgo::Population<T>& x)
{
   int popsize = x.popsize();
   static std::vector<int> rank(popsize);
   static int ranksum;

   // this will only be run at the first generation
   if (x.nogen() == 1) {
      int n = popsize + 1;
      // generating ranks from highest to lowest
      std::generate_n(rank.begin(), popsize, [&n]()->int{return --n;});
      // computing sum of ranks
      ranksum = .5 * popsize * (popsize + 1);
   }

   // selecting mating population
   for (int i = 0, end = x.matsize(); i < end; ++i) {
      // generating a random rank sum in [1,ranksum)
      int rsum = galgo::uniform<int>(1, ranksum);

      int j = 0;
      while (rsum > 0) {
         #ifndef NDEBUG
         if (j == popsize) {
            throw std::invalid_argument("Error: in RNK(galgo::Population<T>&) index j cannot be equal to population size.");
         }
         #endif
         rsum -= rank[j];
         j++;
      }
      // selecting element
      x.select(j - 1);
   }
}

/*-------------------------------------------------------------------------------------------------*/

// linear rank-based selection with selective pressure
template <typename T>
void RSP(galgo::Population<T>& x)
{
   int popsize = x.popsize();
   static std::vector<T> rank(popsize);
   static T ranksum;

   // this will only be run at the first generation
   if (x.nogen() == 1) {
      // initializing ranksum
      ranksum = 0.0;
      // generating ranks from highest to lowest
      for (int i = 0; i < popsize; ++i) {
         rank[i] = 2 - x.SP() + 2 * (x.SP() - 1) * (popsize - i) / popsize;
         ranksum += rank[i];
      }
   }

   // selecting mating population
   for (int i = 0, end = x.matsize(); i < end; ++i) {
      // generating a random rank sum in [0,ranksum)
      T rsum = galgo::uniform<T>(0.0, ranksum);

      int j = 0;
      while (rsum >= 0.0) {
         #ifndef NDEBUG
         if (j == popsize) {
            throw std::invalid_argument("Error: in RSP(galgo::Population<T>&) index j cannot be equal to population size.");
         }
         #endif
         rsum -= rank[j];
         j++;
      }
      // selecting element
      x.select(j - 1);
   }
}

/*-------------------------------------------------------------------------------------------------*/

// tournament selection
template <typename T>
void TNT(galgo::Population<T>& x)
{
   int popsize = x.popsize();
   int tntsize = x.tntsize();

   // selecting mating population
   for (int i = 0, end = x.matsize(); i < end; ++i) {
      // selecting randomly a first element
      int bestIdx = galgo::uniform<int>(0, popsize);
      T bestFit = x(bestIdx)->fitness;

      // starting tournament
      for (int j = 1; j < tntsize; ++j) {

         int idx = galgo::uniform<int>(0, popsize);
         T fit = x(idx)->fitness;

         if (fit > bestFit) {
            bestFit = fit;
            bestIdx = idx;
         }
      }
      // selecting element
      x.select(bestIdx);
   }
}

/*-------------------------------------------------------------------------------------------------*/

// transform ranking selection
template <typename T>
void TRS(galgo::Population<T>& x)
{
   static T c;
   // (re)initializing when running new GA
   if (x.nogen() == 1) {
      c = 0.2;
   }
   int popsize = x.popsize();
   // generating a random set of popsize values on [0,1)
   std::vector<T> r(popsize);
   std::for_each(r.begin(),r.end(),[](T& z)->T{z = galgo::proba(galgo::rng);});
   // sorting them from highest to lowest
   std::sort(r.begin(),r.end(),[](T z1, T z2)->bool{return z1 > z2;});
   // transforming population fitness
   auto it = x.begin();
   std::for_each(r.begin(),r.end(),[&it,popsize](T z)->void{(*it)->fitness = ceil((popsize - popsize*exp(-c*z))/(1 - exp(-c))); it++;});

   // updating c for next generation
   c = c + 0.1; // arithmetic transition
   //c = c * 1.1; // geometric transition
   // computing fitness sum
   int fitsum = x.getSumFitness();

   // selecting mating population
   for (int i = 0, end = x.matsize(); i < end; ++i) {
      // generating a random fitness sum in [0,fitsum)
      T fsum = galgo::uniform<int>(0, fitsum);

      int j = 0;
      while (fsum >= 0) {
         #ifndef NDEBUG
         if (j == popsize) {
            throw std::invalid_argument("Error: in TRS(galgo::Population<T>&) index j cannot be equal to population size.");
         }
         #endif
         fsum -= x(j)->fitness;
         j++;
      }
      // selecting element
      x.select(j - 1);
   }
}

/*-------------------------------------------------------------------------------------------------*/

// CROSS-OVER METHODS

/*-------------------------------------------------------------------------------------------------*/

// one-point random cross-over of 2 chromosomes
template <typename T>
void P1XO(const galgo::Population<T>& x, galgo::CHR<T>& chr1, galgo::CHR<T>& chr2)
{
   // choosing randomly 2 chromosomes from mating population
   int idx1 = galgo::uniform<int>(0, x.matsize());
   int idx2 = galgo::uniform<int>(0, x.matsize());
   // choosing randomly a position for cross-over
   int pos = galgo::uniform<int>(0, chr1->size());
   // transmitting portion of bits to new chromosomes
   chr1->setPortion(*x[idx1], 0, pos);
   chr2->setPortion(*x[idx2], 0, pos);
   chr1->setPortion(*x[idx2], pos + 1);
   chr2->setPortion(*x[idx1], pos + 1);
}

/*-------------------------------------------------------------------------------------------------*/

// two-point random cross-over of 2 chromosomes
template <typename T, int...N>
void P2XO(const galgo::Population<T>& x, galgo::CHR<T>& chr1, galgo::CHR<T>& chr2)
{
   // choosing randomly 2 chromosomes from mating population
   int idx1 = galgo::uniform<int>(0, x.matsize());
   int idx2 = galgo::uniform<int>(0, x.matsize());
   // choosing randomly 2 positions for cross-over
   int pos1 = galgo::uniform<int>(0, chr1->size());
   int pos2 = galgo::uniform<int>(0, chr1->size());
   // ordering these 2 random positions
   int m = std::min(pos1,pos2);
   int M = std::max(pos1,pos2);
   // transmitting portion of bits new chromosomes
   chr1->setPortion(*x[idx1], 0, m);
   chr2->setPortion(*x[idx2], 0, m);
   chr1->setPortion(*x[idx2], m + 1, M);
   chr2->setPortion(*x[idx1], m + 1, M);
   chr1->setPortion(*x[idx1], M + 1);
   chr2->setPortion(*x[idx2], M + 1);
}

/*-------------------------------------------------------------------------------------------------*/

// uniform random cross-over of 2 chromosomes
template <typename T>
void UXO(const galgo::Population<T>& x, galgo::CHR<T>& chr1, galgo::CHR<T>& chr2)
{
   // choosing randomly 2 chromosomes from mating population
   int idx1 = galgo::uniform<int>(0, x.matsize());
   int idx2 = galgo::uniform<int>(0, x.matsize());

   for (int j = 0; j < chr1->size(); ++j) {
      // choosing 1 of the 2 chromosomes randomly
      if (galgo::proba(galgo::rng) < 0.50) {
         // adding its jth bit to new chromosome
         chr1->addBit(x[idx1]->getBit(j));
         chr2->addBit(x[idx2]->getBit(j));
      } else {
         // adding its jth bit to new chromosomes
         chr1->addBit(x[idx2]->getBit(j));
         chr2->addBit(x[idx1]->getBit(j));
      }
   }
}

/*-------------------------------------------------------------------------------------------------*/

// MUTATION METHODS

/*-------------------------------------------------------------------------------------------------*/

// boundary mutation: replacing a chromosome gene by its lower or upper bound
template <typename T>
void BDM(galgo::CHR<T>& chr)
{
   T mutrate = chr->mutrate();

   if (mutrate == 0.0) return;

   // getting chromosome lower bound(s)
   const std::vector<T>& lowerBound = chr->lowerBound();
   // getting chromosome upper bound(s)
   const std::vector<T>& upperBound = chr->upperBound();

   // looping on number of genes
   for (int i = 0; i < chr->nbgene(); ++i) {
      // generating a random probability
      if (galgo::proba(galgo::rng) <= mutrate) {
         // generating a random probability
         if (galgo::proba(galgo::rng) < .5) {
            // replacing ith gene by lower bound
            chr->initGene(i, lowerBound[i]);
         } else {
            // replacing ith gene by upper bound
            chr->initGene(i, upperBound[i]);
         }
      }
   }
}

/*-------------------------------------------------------------------------------------------------*/

// single point mutation: flipping a chromosome bit
template <typename T>
void SPM(galgo::CHR<T>& chr)
{
   T mutrate = chr->mutrate();

   if (mutrate == 0.0) return;

   // looping on chromosome bits
   for (int i = 0; i < chr->size(); ++i) {
      // generating a random probability
      if (galgo::proba(galgo::rng) <= mutrate) {
         // flipping ith bit
         chr->flipBit(i);
      }
   }
}

/*-------------------------------------------------------------------------------------------------*/

// uniform mutation: replacing a chromosome gene by a new one
template <typename T>
void UNM(galgo::CHR<T>& chr)
{
   T mutrate = chr->mutrate();

   if (mutrate == 0.0) return;

   // looping on number of genes
   for (int i = 0; i < chr->nbgene(); ++i) {
      // generating a random probability
      if (galgo::proba(galgo::rng) <= mutrate) {
         // replacing ith gene by a new one
         chr->setGene(i);
      }
   }
}

/*-------------------------------------------------------------------------------------------------*/

// ADAPTATION TO CONSTRAINT(S) METHODS

/*-------------------------------------------------------------------------------------------------*/

// adapt population to genetic algorithm constraint(s)
template <typename T>
void DAC(galgo::Population<T>& x)
{
   // getting worst population objective function total result
   T worstTotal = x.getWorstTotal();

   for (auto it = x.begin(), end = x.end(); it != end; ++it) {
      // computing element constraint value(s)
      const std::vector<T>& cst = (*it)->getConstraint();
      // adapting fitness if any constraint violated
      if (std::any_of(cst.cbegin(), cst.cend(), [](T x)->bool{return x >= 0.0;})) {
         (*it)->fitness = worstTotal - std::accumulate(cst.cbegin(), cst.cend(), 0.0);
      }
   }
}

//=================================================================================================


///common + param


//=================================================================================================

// convert unsigned long long integer to binary string
std::string GetBinary(uint64_t value)
{
   std::bitset<sizeof(uint64_t)*CHAR_BIT> bits(value);
   // NB: CHAR_BIT = number of bits in char usually 8 but not always on older machines
   return bits.to_string();
}

/*-------------------------------------------------------------------------------------------------*/

// convert binary string to unsigned long long integer
uint64_t GetValue(const std::string& s)
{
   uint64_t value, x = 0;
   for (std::string::const_iterator it = s.begin(), end = s.end(); it != end; ++it) {
      x = (x << 1) + (*it - '0');
   }
   memcpy(&value, &x, sizeof(uint64_t));

   return value;
}

// encoding random unsigned integer
template <typename T, int N>
std::string Parameter<T,N>::encode() const  {
   std::string str = GetBinary(galgo::Randomize<N>::generate());
   return str.substr(str.size() - N, N);
}
// encoding known unsigned integer
template <typename T, int N>
std::string Parameter<T,N>::encode(T z) const  {
   uint64_t value = Randomize<N>::MAXVAL * (z - data[0]) / (data[1] - data[0]);
   std::string str = GetBinary(value);
   return str.substr(str.size() - N, N);
}
// decoding string to real value
template <typename T, int N>
T Parameter<T,N>::decode(const std::string& str) const  {
   return data[0] + (GetValue(str) / static_cast<double>(Randomize<N>::MAXVAL)) * (data[1] - data[0]);
}



//=================================================================================================

// template metaprogramming for getting maximum unsigned integral value from N bits
template <unsigned int N>
struct MAXVALUE
{
    enum : uint64_t{ value = 2 * MAXVALUE<N - 1>::value };
};

// template specialization for initial case N = 0
template <>
struct MAXVALUE<0>
{
    enum { value = 1 };
};

/*-------------------------------------------------------------------------------------------------*/

// Mersenne Twister 19937 pseudo-random number generator
std::random_device rand_dev;
std::mt19937_64 rng(rand_dev());
// generate uniform random probability in range [0,1)
std::uniform_real_distribution<> proba(0, 1);

/*-------------------------------------------------------------------------------------------------*/

// generate a uniform random number within the interval [min,max)
template <typename T>
inline T uniform(T min, T max)
{
    #ifndef NDEBUG
    if (min >= max) {
      throw std::invalid_argument("Error: in galgo::uniform(T, T), first argument must be < to second argument.");
    }
    #endif

    return min + proba(rng) * (max - min);
}

/*-------------------------------------------------------------------------------------------------*/

// static class for generating random unsigned integral numbers
template <int N>
class Randomize
{
   static_assert(N > 0 && N <= 64, "in class galgo::Randomize<N>, template parameter N cannot be ouside interval [1,64], please choose an integer within this interval.");

public:
   // computation only done once for each different N
   static constexpr uint64_t MAXVAL = MAXVALUE<N>::value - 1;

   // generating random unsigned long long integer on [0,MAXVAL]
   static uint64_t generate() {
      // class constructor only called once for each different N
      static std::uniform_int_distribution<uint64_t> udistrib(0,MAXVAL);
      return udistrib(rng);
   }
};

//=================================================================================================


///common

//=================================================================================================

// end of recursion for computing the sum of a parameter pack of integral numbers
int sum(int first)
{
   return first;
}

// recursion for computing the sum of a parameter pack of integral numbers
template <typename...Args>
int sum(int first, Args...args)
{
   return first + sum(args...);
}

///genetic

/*-------------------------------------------------------------------------------------------------*/

// constructor
template <typename T> template <int...N>
GeneticAlgorithm<T>::GeneticAlgorithm(Func<T> objective, int popsize, int nbgen, bool output, const Parameter<T,N>&...args)
{
   this->Objective = objective;
   // getting total number of bits per chromosome
   this->nbbit = sum(N...);
   this->nbgen = nbgen;
   // getting number of parameters in the pack
   this->nbparam = sizeof...(N);
   this->popsize = popsize;
   this->matsize = popsize;
   this->output = output;
   // unpacking parameter pack in tuple
   TUP<T,N...> tp(args...);
   // initializing parameter(s) data
   this->init(tp);
   ///chq\\\
   Selection= RWS;
   // cross-over method initialized to 1-point cross-over
   CrossOver= P1XO;
   // mutation method initialized to single-point mutation
   Mutation = SPM;
}

/*-------------------------------------------------------------------------------------------------*/

// end of recursion for initializing parameter(s) data
template <typename T> template <int I, int...N>
inline typename std::enable_if<I == sizeof...(N), void>::type
GeneticAlgorithm<T>::init(const TUP<T,N...>& tp) {}

// recursion for initializing parameter(s) data
template <typename T> template <int I, int...N>
inline typename std::enable_if<I < sizeof...(N), void>::type
GeneticAlgorithm<T>::init(const TUP<T,N...>& tp)
{
   // getting Ith parameter in tuple
   auto par = std::get<I>(tp);
   // getting Ith parameter initial data
   const std::vector<T>& data = par.getData();
   // copying parameter data
   param.emplace_back(new decltype(par)(par));
   lowerBound.push_back(data[0]);
   upperBound.push_back(data[1]);
   // if parameter has initial value
   if (data.size() > 2) {
      initialSet.push_back(data[2]);
   }
   // setting indexes for chromosome breakdown
   if (I == 0) {
      idx.push_back(0);
   } else {
      idx.push_back(idx[I - 1] + par.size());
   }
   // recursing
   init<I + 1>(tp);
}

/*-------------------------------------------------------------------------------------------------*/

// check inputs validity
template <typename T>
void GeneticAlgorithm<T>::check() const
{
   if (!initialSet.empty()) {
      for (int i = 0; i < nbparam; ++i) {
         if (initialSet[i] < lowerBound[i] || initialSet[i] > upperBound[i]) {
            throw std::invalid_argument("Error: in class galgo::Parameter<T,N>, initial parameter value cannot be outside the parameter boundaries, please choose a value between its lower and upper bounds.");
         }
      }
      if (initialSet.size() != (unsigned)nbparam) {
         throw std::invalid_argument("Error: in class galgo::GeneticAlgorithm<T>, initial set of parameters does not have the same dimension than the number of parameters, please adjust.");
      }
   }
   if (SP < 1.0 || SP > 2.0) {
      throw std::invalid_argument("Error: in class galgo::GeneticAlgorithm<T>, selective pressure (SP) cannot be outside [1.0,2.0], please choose a real value within this interval.");
   }
   if (elitpop > popsize || elitpop < 0) {
      throw std::invalid_argument("Error: in class galgo::GeneticAlgorithm<T>, elit population (elitpop) cannot outside [0,popsize], please choose an integral value within this interval.");
   }
   if (covrate < 0.0 || covrate > 1.0) {
      throw std::invalid_argument("Error: in class galgo::GeneticAlgorithm<T>, cross-over rate (covrate) cannot outside [0.0,1.0], please choose a real value within this interval.");
   }
   if (genstep <= 0) {
      throw std::invalid_argument("Error: in class galgo::GeneticAlgorithm<T>, generation step (genstep) cannot be <= 0, please choose an integral value > 0.");
   }
}

/*-------------------------------------------------------------------------------------------------*/

// run genetic algorithm
template <typename T>
void GeneticAlgorithm<T>::run()
{
   // checking inputs validity
   this->check();

   // setting adaptation method to default if needed
   if (Constraint != nullptr && Adaptation == nullptr) {
      Adaptation = DAC;
   }

   // initializing population
   pop = Population<T>(*this);

   if (output) {
      std::cout << "\n Running Genetic Algorithm...\n";
      std::cout << " ----------------------------\n";
   }

   // creating population
   pop.creation();
   // initializing best result and previous best result
   T bestResult = pop(0)->getTotal();
   T prevBestResult = bestResult;
   // outputting results
   if (output) print();

   // starting population evolution
   for (nogen = 1; nogen <= nbgen; ++nogen) {
      // evolving population
      pop.evolution();
      // getting best current result
      bestResult = pop(0)->getTotal();
      // outputting results
      if (output) print();
      // checking convergence
      if (tolerance != 0.0) {
         if (fabs(bestResult - prevBestResult) < fabs(tolerance)) {
            break;
         }
         prevBestResult = bestResult;
      }
   }

   // outputting contraint value
   if (Constraint != nullptr) {
      // getting best parameter(s) constraint value(s)
      std::vector<T> cst = pop(0)->getConstraint();
      if (output) {
         std::cout << "\n Constraint(s)\n";
         std::cout << " -------------\n";
         for (unsigned i = 0; i < cst.size(); ++i) {
            std::cout << " C";
            if (nbparam > 1) {
               std::cout << std::to_string(i + 1);
            }
            std::cout << "(x) = " << std::setw(6) << std::fixed << std::setprecision(precision) << cst[i] << "\n";
         }
         std::cout << "\n";
      }
   }
}

/*-------------------------------------------------------------------------------------------------*/

// return best chromosome
template <typename T>
inline const CHR<T>& GeneticAlgorithm<T>::result() const
{
   return pop(0);
}

/*-------------------------------------------------------------------------------------------------*/

// print results for each new generation
template <typename T>
void GeneticAlgorithm<T>::print() const
{
   // getting best parameter(s) from best chromosome
   std::vector<T> bestParam = pop(0)->getParam();
   std::vector<T> bestResult = pop(0)->getResult();

   if (nogen % genstep == 0) {
      std::cout << " Generation = " << std::setw(std::to_string(nbgen).size()) << nogen << " |";
      for (int i = 0; i < nbparam; ++i) {
        std::cout << " X";
         if (nbparam > 1) {
            std::cout << std::to_string(i + 1);
         }
         std::cout << " = "  << std::setw(9) << std::fixed << std::setprecision(precision) << bestParam[i] << " |";
     }
      for (unsigned i = 0; i < bestResult.size(); ++i) {
        std::cout << " F";
         if (bestResult.size() > 1) {
            std::cout << std::to_string(i + 1);
         }
         std::cout << "(x) = " << std::setw(12) << std::fixed << std::setprecision(precision) << bestResult[i];
         if (i < bestResult.size() - 1) {
            std::cout << " |";
         } else {
            std::cout << "\n";
         }
     }

   }
}

//=================================================================================================

///Population

/*-------------------------------------------------------------------------------------------------*/

// constructor
template <typename T>
Population<T>::Population(const GeneticAlgorithm<T>& ga)
{
   ptr = &ga;
   nbrcrov = floor(ga.covrate * (ga.popsize - ga.elitpop));
   // adjusting nbrcrov (must be an even number)
   if (nbrcrov % 2 != 0) nbrcrov -= 1;
   // for convenience, we add elitpop to nbrcrov
   nbrcrov += ga.elitpop;
   // allocating memory
   curpop.resize(ga.popsize);
   matpop.resize(ga.matsize);
}

/*-------------------------------------------------------------------------------------------------*/

// create a population of chromosomes
template <typename T>
void Population<T>::creation()
{
   int start = 0;
   // initializing first chromosome
   if (!ptr->initialSet.empty()) {
      curpop[0] = std::make_shared<Chromosome<T>>(*ptr);
      curpop[0]->initialize();
      curpop[0]->evaluate();
      start++;
   }
   // getting the rest
   #ifdef _OPENMP
   #pragma omp parallel for num_threads(MAX_THREADS)
   #endif
   for (int i = start; i < ptr->popsize; ++i) {
      curpop[i] = std::make_shared<Chromosome<T>>(*ptr);
      curpop[i]->create();
      curpop[i]->evaluate();
   }
   // updating population
   this->updating();
}

/*-------------------------------------------------------------------------------------------------*/

// population evolution (selection, recombination, completion, mutation), get next generation
template <typename T>
void Population<T>::evolution()
{
   // initializing mating population index
   matidx = 0;
   // selecting mating population
   ptr->Selection(*this);
   // applying elitism if required
   this->elitism();
   // crossing-over mating population
   this->recombination();
   // completing new population
   this->completion();
   // moving new population into current population for next generation
   curpop = std::move(newpop);
   // updating population
   this->updating();
}

/*-------------------------------------------------------------------------------------------------*/

// elitism => saving best chromosomes in new population, making a copy of each elit chromosome
template <typename T>
void Population<T>::elitism()
{
   // (re)allocating new population
   newpop.resize(ptr->popsize);

   if (ptr->elitpop > 0) {
      // copying elit chromosomes into new population
      std::transform(curpop.cbegin(), curpop.cend(), newpop.begin(), [](const CHR<T>& chr)->CHR<T>{return std::make_shared<Chromosome<T>>(*chr);});
   }
}

/*-------------------------------------------------------------------------------------------------*/

// create new population from recombination of the old one
template <typename T>
void Population<T>::recombination()
{
   // creating a new population by cross-over
   #ifdef _OPENMP
   #pragma omp parallel for num_threads(MAX_THREADS)
   #endif
   for (int i = ptr->elitpop; i < nbrcrov; i = i + 2) {
      // initializing 2 new chromosome
      newpop[i] = std::make_shared<Chromosome<T>>(*ptr);
      newpop[i+1] = std::make_shared<Chromosome<T>>(*ptr);
      // crossing-over mating population to create 2 new chromosomes
      ptr->CrossOver(*this, newpop[i], newpop[i+1]);
      // mutating new chromosomes
      ptr->Mutation(newpop[i]);
      ptr->Mutation(newpop[i+1]);
      // evaluating new chromosomes
      newpop[i]->evaluate();
      newpop[i+1]->evaluate();
   }
}

/*-------------------------------------------------------------------------------------------------*/

// complete new population
template <typename T>
void Population<T>::completion()
{
   #ifdef _OPENMP
   #pragma omp parallel for num_threads(MAX_THREADS)
   #endif
   for (int i = nbrcrov; i < ptr->popsize; ++i) {
      // selecting chromosome randomly from mating population
      newpop[i] = std::make_shared<Chromosome<T>>(*matpop[uniform<int>(0, ptr->matsize)]);
      // mutating chromosome
      ptr->Mutation(newpop[i]);
      // evaluating chromosome
      newpop[i]->evaluate();
   }
}

/*-------------------------------------------------------------------------------------------------*/

// update population (adapting, sorting)
template <typename T>
void Population<T>::updating()
{
   // adapting population to constraints
   if (ptr->Constraint != nullptr) {
      ptr->Adaptation(*this);
   }
   // sorting chromosomes from best to worst fitness
   std::sort(curpop.begin(),curpop.end(),[](const CHR<T>& chr1,const CHR<T>& chr2)->bool{return chr1->fitness > chr2->fitness;});
}

/*-------------------------------------------------------------------------------------------------*/

// access element in current population at position pos
template <typename T>
const CHR<T>& Population<T>::operator()(int pos) const
{
   #ifndef NDEBUG
   if (pos > ptr->popsize - 1) {
      throw std::invalid_argument("Error: in galgo::Population<T>::operator()(int), exceeding current population memory.");
   }
   #endif

   return curpop[pos];
}

/*-------------------------------------------------------------------------------------------------*/

// access element in mating population at position pos
template <typename T>
const CHR<T>& Population<T>::operator[](int pos) const
{
   #ifndef NDEBUG
   if (pos > ptr->matsize - 1) {
      throw std::invalid_argument("Error: in galgo::Population<T>::operator[](int), exceeding mating population memory.");
   }
   #endif

   return matpop[pos];
}

/*-------------------------------------------------------------------------------------------------*/

// return iterator to current population beginning
template <typename T>
inline typename std::vector<CHR<T>>::iterator Population<T>::begin()
{
   return curpop.begin();
}

/*-------------------------------------------------------------------------------------------------*/

// return const iterator to current population beginning
template <typename T>
inline typename std::vector<CHR<T>>::const_iterator Population<T>::cbegin() const
{
   return curpop.cbegin();
}

/*-------------------------------------------------------------------------------------------------*/

// return iterator to current population ending
template <typename T>
inline typename std::vector<CHR<T>>::iterator Population<T>::end()
{
   return curpop.end();
}

/*-------------------------------------------------------------------------------------------------*/

// return const iterator to current population ending
template <typename T>
inline typename std::vector<CHR<T>>::const_iterator Population<T>::cend() const
{
   return curpop.cend();
}

/*-------------------------------------------------------------------------------------------------*/

// select element at position pos in current population and copy it into mating population
template <typename T>
inline void Population<T>::select(int pos)
{
   #ifndef NDEBUG
   if (pos > ptr->popsize - 1) {
      throw std::invalid_argument("Error: in galgo::Population<T>::select(int), exceeding current population memory.");
   }
   if (matidx == ptr->matsize) {
      throw std::invalid_argument("Error: in galgo::Population<T>::select(int), exceeding mating population memory.");
   }
   #endif

   matpop[matidx] = curpop[pos];
   matidx++;
}

/*-------------------------------------------------------------------------------------------------*/

// set all fitness to positive values (used in RWS and SUS selection methods)
template <typename T>
void Population<T>::adjustFitness()
{
   // getting worst population fitness
   T worstFitness = curpop.back()->fitness;

   if (worstFitness < 0) {
      // getting best fitness
      T bestFitness = curpop.front()->fitness;
      // case where all fitness are equal and negative
      if (worstFitness == bestFitness) {
         std::for_each(curpop.begin(), curpop.end(), [](CHR<T>& chr)->void{chr->fitness *= -1;});
       } else {
         std::for_each(curpop.begin(), curpop.end(), [worstFitness](CHR<T>& chr)->void{chr->fitness -= worstFitness;});
      }
   }
}

/*-------------------------------------------------------------------------------------------------*/

// compute population fitness sum (used in TRS, RWS and SUS selection methods)
template <typename T>
inline T Population<T>::getSumFitness() const
{
   return std::accumulate(curpop.cbegin(), curpop.cend(), 0.0, [](T sum, const CHR<T>& chr)->T{return sum + T(chr->fitness);});
}

/*-------------------------------------------------------------------------------------------------*/

// get worst objective function total result from current population (used in constraint(s) adaptation)
template <typename T>
inline T Population<T>::getWorstTotal() const
{
   auto it = std::min_element(curpop.begin(), curpop.end(), [](const CHR<T>& chr1, const CHR<T>& chr2)->bool{return chr1->getTotal() < chr2->getTotal();});

   return (*it)->getTotal();
}

/*-------------------------------------------------------------------------------------------------*/

// return population size
template <typename T>
inline int Population<T>::popsize() const
{
   return ptr->popsize;
}

/*-------------------------------------------------------------------------------------------------*/

// return mating population size
template <typename T>
inline int Population<T>::matsize() const
{
   return ptr->matsize;
}

/*-------------------------------------------------------------------------------------------------*/

// return tournament size
template <typename T>
inline int Population<T>::tntsize() const
{
   return ptr->tntsize;
}

/*-------------------------------------------------------------------------------------------------*/

// return numero of generation
template <typename T>
inline int Population<T>::nogen() const
{
   return ptr->nogen;
}

/*-------------------------------------------------------------------------------------------------*/

// return number of generations
template <typename T>
inline int Population<T>::nbgen() const
{
   return ptr->nbgen;
}

/*-------------------------------------------------------------------------------------------------*/

// return selection pressure
template <typename T>
inline T Population<T>::SP() const
{
   return ptr->SP;
}

//=================================================================================================


}


#endif // GALGO_CPP

