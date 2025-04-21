#ifndef INC_RING_BUFFER_H_
#define INC_RING_BUFFER_H_

#include <cstdint>
#include <stdexcept>

template <typename T, uint8_t Dimensions = 2>
class RingBuffer {

    public:
        /* コンストラクタ
        * @param[in] rows: 行数（1次元の場合は全体の要素数）
        * @param[in] cols: 列数（2次元の場合のみ使用）
        */
        RingBuffer(uint8_t rows, uint8_t cols = 1) {
            rows_count = rows;
            cols_count = cols;
            if (Dimensions == 1) {
                max_index = rows - 1;
                buffer = new T[rows];
            }
            else if (Dimensions == 2) {
                max_index = rows - 1;
                buffer_2d = new T*[rows];

                for(uint8_t i = 0; i < rows; ++i){
                    buffer_2d[i] = new T[cols];
                }
            }

            index = 0;
        }

        // デストラクタ
        ~RingBuffer() {
            if (Dimensions == 1){
                delete[] buffer;
            }
            else if (Dimensions == 2){
                for (uint8_t i = 0; i <= max_index; ++i){
                    delete[] buffer_2d[i];
                }
                delete[] buffer_2d;
            }
        }

        /* データの入力（1次元）
        * @param[in] value: 入力するデータ
        */
        void SetValue(const T& value) {
            buffer[index] = value;

            //indexが最大の場合は0に戻す
            if(index == max_index){
                index = 0;
            }
            else{
                index ++;
            }
        }

        /* データの入力（2次元）
        * @param[in] row: 行インデックス
        * @param[in] col: 列インデックス
        * @param[in] value: 入力するデータ
        */
        void SetValue(uint8_t row, uint8_t col, const T& value) {
            buffer_2d[row][col] = value;
        }

        /* データの入力（2次元、1行分）
        * @param[in] value: 入力するデータ（1行分の配列）
        */
        template <uint8_t N>
        void SetValue(const T (&value)[1][N]) {
            for (uint8_t i = 0; i < cols_count; ++i) {
                buffer_2d[index][i] = value[0][i];
            }
            if (index == max_index) {
                index = 0;
            } else {
                index++;
            }
        }

        // 2次元バッファに1次元配列を入力する
        void SetValue(const T* value) {
            for (uint8_t i = 0; i < cols_count; ++i) {
                buffer_2d[index][i] = value[i];
            }
            if (index == max_index) {
                index = 0;
            } else {
                index++;
            }
        }

        /* データの取得（1次元）
        * @param[in] idx: インデックス
        * @return データ
        */
        T GetValue(uint8_t idx) const {
            return buffer[idx];
        }

        /* データの取得（2次元）
        * @param[in] row: 行インデックス
        * @param[in] col: 列インデックス
        * @return データ
        */
        T GetValue(uint8_t row, uint8_t col) const {
            return buffer_2d[row][col];
        }

        /* 最新のデータを取得（1次元）
        * @param[in] n: 取得するデータの個数
        * @return データの配列
        */
        void GetLatestValues(T* data_buffer, uint8_t n) const {
            for (uint8_t i = 0; i < n; ++i) {
                uint8_t idx = (index + max_index + 1 - n + i) % (max_index + 1);
                data_buffer[i] = buffer[idx];
            }
        }

        /* 最新のデータを取得（2次元）
        * @param[in] n: 取得する行数
        * @return void
        */
        void GetLatestValues2D(T** data_buffer, uint8_t n) const {
            for (uint8_t i = 0; i < n; ++i) {
                uint8_t row_idx = (index + max_index + 1 - n + i) % (max_index + 1);
                data_buffer[i] = new T[cols_count];
                for (uint8_t j = 0; j < cols_count; ++j) {
                    data_buffer[i][j] = buffer_2d[row_idx][j];
                }
            }
        }

    private:
        uint8_t index;
        uint8_t max_index;
        uint8_t rows_count;       // 追加
        uint8_t cols_count = 1;   // 既存
        T* buffer = nullptr;    // 1次元配列用
        T** buffer_2d = nullptr; // 2次元配列用
};

#endif /* INC_RING_BUFFER_H_ */
