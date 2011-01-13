/*
 * Copyright (c) 2011, Regents of the University of Michigan
 * 
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 * 
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 * 
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWARE.
 */
package edu.umich.robot.radio;

import java.util.ArrayList;
import java.util.Collections;
import java.util.Iterator;
import java.util.List;
import java.util.concurrent.atomic.AtomicLong;

/**
 * Immutable.
 * 
 * @author voigtjr
 */
public class RadioMessage
{
    private static final AtomicLong counter = new AtomicLong();

    static RadioMessage newCommunication(String from, String destination,
            List<String> tokens)
    {
        return new RadioMessage(from, destination, tokens);
    }

    static RadioMessage newCommunication(String from, List<String> tokens)
    {
        return newCommunication(from, null, tokens);
    }

    public static class Builder
    {
        private final String from;

        private final List<String> tokens = new ArrayList<String>();

        private String destination;

        public Builder(String from)
        {
            this.from = from;
        }

        public Builder destination(String destination)
        {
            this.destination = destination;
            return this;
        }

        public Builder token(String token)
        {
            tokens.add(token);
            return this;
        }

        public RadioMessage build()
        {
            Iterator<String> iter = tokens.iterator();
            while (iter.hasNext())
            {
                String token = iter.next();
                if (token.length() == 0)
                {
                    iter.remove();
                }
            }

            if (tokens.isEmpty())
                return null;

            return new RadioMessage(from, destination, tokens);
        }
    }

    private final long id;

    private final String from;

    private final List<String> tokens;

    private final String destination;

    public RadioMessage(String from, String destination, List<String> tokens)
    {
        this.id = counter.getAndIncrement();
        this.from = from;
        this.destination = destination;
        this.tokens = Collections
                .unmodifiableList(new ArrayList<String>(tokens));
    }

    public String getDestination()
    {
        return destination;
    }

    public String getFrom()
    {
        return from;
    }

    public long getId()
    {
        return id;
    }

    public List<String> getTokens()
    {
        return tokens;
    }

    public boolean isBroadcast()
    {
        return destination == null;
    }
}
