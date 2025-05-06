/*
 * RingBuffer.h
 *
 *  Created on: Apr 22, 2025
 *      Author: Sezakiaoi
 */

#ifndef INC_DATABUFFER_HPP_
#define INC_DATABUFFER_HPP_

#include <cstdint>

template<typename T>
class DATABUFFER {

    public:
        /* @brief コンストラクタ
         * @param [in]dim データの次元数
         * @param [in]size バッファのサイズ
         * @note バッファはdim*sizeのサイズで確保される
         */
		DATABUFFER(uint8_t dim, uint8_t size){
            this->dim = dim;
            this->size = size;
            this->index = 0;
            buffer = new T[dim * size];
        }

        /* @brief デストラクタ
         * @note バッファのメモリを解放する
         */
        ~DATABUFFER(){
            delete[] buffer;
        }

        /* @brief データをセットする
         * @param [in]data セットするデータのポインタ
         * @note データは[dim][1]の配列である必要がある(T*型になる)
         * @note データはバッファのサイズを超えないようにする必要がある
         */
        void SetData(const T* data){

            for(uint8_t i = 0; i < dim; i++){
                buffer[index * dim + i] = data[i];
            }
            index = (index + 1) % size;
        }

        /* @brief データを取得する
         * @param [out]data 取得したデータのポインタ
         * @param [in]n 取得するデータの数
         * @note データはdim次元の配列である必要がある
         * @note データはバッファのサイズを超えないようにする必要がある
         */
        void GetData(T* data, uint8_t n) const {
            for(uint8_t k = 0; k < n; k++) {

                uint8_t idx;

                if(index == 0) {
                    idx = (size - k - 1 + size) % size;
                } 
                else if(index > k) {
                    idx = index - k - 1;
                } 
                else {
                    idx = size + index - k - 1;
                }

                for(uint8_t i = 0; i < dim; i++) {

                    data[k * dim + i] = buffer[idx * dim + i];
                }
            }
        }

    private:

        T* buffer;
        uint8_t dim;
        uint8_t size;
        uint8_t index;
};

 #endif /* INC_DATABUFFER_HPP_ */
