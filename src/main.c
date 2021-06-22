#include "stm32l0xx.h"
#include "stm32l0xx_nucleo.h"

//-----------global data--------------//
//Tablica 32 elementowa
extern uint8_t coss[];

uint8_t g_lastStableState = 1;
uint8_t g_lastState = 1;
uint8_t g_lastStateCount = 0;
const uint8_t COUNT_MAX = 10;

uint8_t N = 0; // Kolejne częstotliwości -> N należy do zakresu <0,5> odpowiada to mnożnikom 0,1,2,4,8,16
uint8_t h = 0; // krok w tablicy

uint16_t k = 0; // wartosc dla funkcji PWM, zakres <0,511>
uint16_t TempT = 0; // zmienna pomocnicza licząca czas wcisnięcia przycisku
uint8_t cT = 0; // zmienna pomocnicza decydująca o zmianie kroku w tablicy

//------------Funkcje--------------//

//Mamy ztablicowaną cwiartkę cosinusa. Funkcja ta odpowiada za odczytywanie z innych cwiartek oraz zwraca wartosc funkcji wedlug której ma świecic dioda * 2.
//Razy 2 żeby nie było zbędnego dzielenssia - Problem ten rozwiązany został przez zwiększenie Arr Timera od PWM dwukrotnie.

int wartosc_funkcji() {
	static uint16_t T = 32 * 4;
	static uint16_t wynik = 0;

	if (h > T)
		h = 0;

	if (0 <= h && h < T / 4) {

		wynik = 256 + (coss[h] + 1);

	}
	if (T / 4 <= h && h < T / 2) {

		wynik = (256 + (-coss[T / 2 - h] + 1)); //jak cos to ew. jeszcze -1
	}
	if (T / 2 <= h && h < 3 * T / 4) {

		wynik = (256 + (-coss[h - T / 2] + 1));
	}
	if (3 * T / 4 <= h && h <= T) {

		wynik = (256 + (coss[T - h] + 1));
	}

	return wynik;

}
// Funkcja nastawiająca wypłenienie PWM
void setPWM_value(uint16_t value) {
	TIM2->CCR1 = value;
}

// Funkcja przerwania od Timera 2
void TIM21_IRQHandler(void) // periodic @ 1.25ms
{
	cT++;

	static uint8_t currentStableState;

// debouncing
	if (0 != (TIM_SR_UIF & TIM21->SR)) { // check update interrupt flag
		TIM21->SR &= ~TIM_SR_UIF; // clear interrupt flag

		uint8_t currentState = (GPIO_IDR_ID13 & GPIOC->IDR) ? (1) : (0);

		if (currentState != g_lastState) {
			g_lastState = currentState;
			g_lastStateCount = 0;

		} else /* (currentState == g_lastState) */{
			if (COUNT_MAX >= g_lastStateCount) {
				g_lastStateCount++;
			}
			if (COUNT_MAX == g_lastStateCount) {
				currentStableState = currentState;

				g_lastStableState = currentStableState;
			}
		}
	}

// logika przycisku
	if (0 == currentStableState)
		TempT++;
	else {
		if (TempT >= 2048) {
			N = 0;
			h = 0;
		} else if (TempT >= 20 && N < 5) {
			N++;

		}

		TempT = 0;
	}

	if (N != 0 && 0 == cT % 16) {
		switch (N) {
		case 1: {
			h = h + 1;
			break;
		}
		case 2: {
			h = h + 2;
			break;
		}
		case 3: {
			h = h + 4;
			break;
		}
		case 4: {
			h = h + 8;
			break;
		}
		case 5:
			h = h + 16;
			break;
		}
		k = wartosc_funkcji() - 1;
		setPWM_value(k);
		cT = 0;
	} else if (0 == N && 0 == cT % 16) {
		k = 511;
		setPWM_value(k);
		cT = 0;
	}

}

//---------------MAIN------------------//
int main(void) {

	RCC->CR |= RCC_CR_HSION; // Włączamy HSI
	while (0 == (RCC->CR & RCC_CR_HSIRDY)) { //Czekamy na włączenie HSI
		//
	}

	RCC->CFGR |= RCC_CFGR_SW_0; // ustawiamy SYSCLK na HSI16

	RCC->IOPENR |= RCC_IOPENR_GPIOCEN | RCC_IOPENR_GPIOAEN; // Aktywacja GPIOA I C

	// TIMER 21 - Czas
	RCC->APB2ENR |= RCC_APB2ENR_TIM21EN; // Timer 21 włączony
	TIM21->CR1 &= ~TIM_CR1_DIR; //Zliczanie do góry.
	TIM21->ARR = 625 - 1; // 1024 Hz
	TIM21->PSC = 25 - 1; // 1024 Hz
	TIM21->CR1 |= TIM_CR1_CEN; // Aktywacja licznika
	TIM21->CR1 |= TIM_CR1_ARPE; //Auto-reload preload enable register is buffered.
	TIM21->DIER |= TIM_DIER_UIE;
	NVIC_EnableIRQ(TIM21_IRQn);
	// Konfiguracja diody i przycisku
	GPIOA->MODER &= ~(GPIO_MODER_MODE5_1 | GPIO_MODER_MODE5_0); // Alternate mode
	GPIOA->MODER |= GPIO_MODER_MODE5_1; // Alternate mode
	GPIOA->OSPEEDR |= (GPIO_OSPEEDER_OSPEED5_1 | GPIO_OSPEEDER_OSPEED5_0); //
	GPIOC->MODER &= ~(GPIO_MODER_MODE13_1 | GPIO_MODER_MODE13_0); // input mode

	GPIOA->AFR[0] &= ~0x00F00000;
	GPIOA->AFR[0] |= 0x00500000; //Podpięcie diody pod sygnał PWM wysyłany z TIM2

	// TIMER 2 -  PWM

	RCC->APB1ENR |= RCC_APB1ENR_TIM2EN;
	TIM2->CCMR1 |= TIM_CCMR1_OC1PE; //Output compare enable
	TIM2->CR1 |= TIM_CR1_ARPE; // Auto-reload preload enable register is buffered.
	TIM2->CR1 &= ~TIM_CR1_DIR; //Zliczanie do góry.
	TIM2->ARR = 512 - 1; // 125 Hz
	TIM2->PSC = 250 - 1; // 125 Hz
	TIM2->CR1 |= TIM_CR1_CEN; // Aktywacja licznika
	TIM2->DIER |= TIM_DIER_UIE;
	TIM2->CCMR1 &= (TIM_CCMR1_OC1M_1 | TIM_CCMR1_OC1M_2 | TIM_CCMR1_OC1M_0);
	TIM2->CCMR1 |= (TIM_CCMR1_OC1M_2 | TIM_CCMR1_OC1M_1); //PWM MODE
	TIM2->CCER |= TIM_CCER_CC1E_Msk;
	TIM2->EGR |= (1 << TIM_EGR_UG_Pos); // przepisanie konfiguracji do shadow registers modułu PWM

	TIM2->CR1 |= TIM_CR1_CEN_Msk;

	while (1) {

	}
}

