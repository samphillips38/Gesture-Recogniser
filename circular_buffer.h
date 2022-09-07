#ifndef CIRCULAR_BUFFER
#define CIRCULAR_BUFFER

#include <Arduino.h>
#include <iostream>
#include <vector>

class Node {
    public:
        Node() {
            x = 0.0;
            y = 0.0;
            z = 0.0;
            next = NULL;
        }
        double x;
        double y;
        double z;
        Node* next;

        /// Print out the node
        void print() {
            Serial.print("[");
            Serial.print(x);
            Serial.print(", ");
            Serial.print(y);
            Serial.print(", ");
            Serial.print(z);
            Serial.print("]");
        }
};

class CircularBuffer {
    private:
      int len = 0;
    public:
        CircularBuffer(int n) {
            // Initialise
            len = n;
            head = new Node();
            tail = new Node();

            // Create Nodes (filled in with zeros)
            Node* current_node = head;
            Node* next_node = head;
            for (int i=0; i<n; i++) {
                next_node = new Node();
                current_node->next = next_node;
                current_node = next_node;
            }
            current_node->next = head->next;
            tail->next = current_node;
        }
        Node* head;
        Node* tail;

        /// Set length NOT FINISHED
        void set_length(int n) {
          if (n < 0) {
            Serial.println("Cannot set negative length!");
            return;
          }
          while (len > n) {
            tail->next->next = head->next->next;
            head->next = head->next->next;
          }
        }

        /// Print out the list from most last to most recent
        void print() {
            Node* this_node = head->next;
            Serial.print("\n[");
            while (this_node->next != head->next) {
                this_node->print();
                Serial.print("\n");
                this_node = this_node->next;
            }
            // Print last value
            this_node->print();
            Serial.print("]\n");
        }

        /// Add element to the circular buffer
        void add(double x, double y, double z) {
            head->next->x = x;
            head->next->y = y;
            head->next->z = z;
            tail->next = head->next;
            head->next = head->next->next;
        }

        /// Return a pointer to a vector representation of the circular buffer
        // vector<double>* getVector() {
        //     vector<double>* output_vector = new vector<double>;
        //     Node* this_node = head->next;
        //     while (this_node->next != head->next) {
        //         output_vector->push_back(this_node->value);
        //         this_node = this_node->next;
        //     }
        //     output_vector->push_back(this_node->value);
        //     return output_vector;
        // }
};

#endif
